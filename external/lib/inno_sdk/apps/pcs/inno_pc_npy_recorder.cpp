  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/inno_pc_npy_recorder.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <string>

#include "src/sdk_common/inno_lidar_api.h"
#include "src/sdk_common/inno_lidar_packet_utils.h"
#include "src/utils/inno_lidar_log.h"

namespace innovusion {

InnoPcNpyRecorder::InnoPcNpyRecorder(const std::string &f,
                                     ssize_t size_limit_in_m,
                                     bool record_npy,
                                     bool record_xyz) {
  filename_ = f;
  size_limit_ = size_limit_in_m * 1000 * 1000;
  record_npy_ = record_npy;
  record_xyz_ = record_xyz;
  if (record_npy_) {
    inno_log_verify(!record_xyz_,
                    "cannot record xyz in npy format");
  }
  if (filename_.rfind(".") == std::string::npos) {
    if (record_npy_) {
      filename_ += ".inno_pc_npy";
    } else {
      if (record_xyz) {
        filename_ += ".inno_pc_xyz";
      } else {
        filename_ += ".inno_pc";
      }
    }
  }

  fd_ = open(filename_.c_str(), O_WRONLY
             | O_CREAT | O_TRUNC, 0644);
  if (fd_ < 0) {
    inno_log_error_errno("cannot open inno pc recorder %s.",
                         filename_.c_str());
  }
}

InnoPcNpyRecorder::~InnoPcNpyRecorder() {
  flush_buffer_();
  close_file_();
}

void InnoPcNpyRecorder::close_file_() {
  if (fd_ >= 0) {
    if (filename_.size()) {
      if (record_npy_) {
        inno_log_info("write %lu frames, %lu points, %ld bytes to %s",
                      total_frame_, total_points_,
                      total_size_, filename_.c_str());
      } else {
        inno_log_info("write %ld bytes to %s",
                      total_size_, filename_.c_str());
      }
    }
    close(fd_);
    fd_ = -1;
  }
}

int InnoPcNpyRecorder::flush_buffer_() {
  inno_log_verify(buffer_write_cursor_ <= sizeof(pt_buffer_),
                  "buffer overflow %lu", buffer_write_cursor_);
  if (buffer_write_cursor_ == 0) {
    return RERCORDER_SUCCESS;
  }
  if (is_opened()) {
    int ret = write(fd_, pt_buffer_, buffer_write_cursor_);
    if (ret < ssize_t(buffer_write_cursor_)) {
      inno_log_error_errno("write %s failed %d",
                           filename_.c_str(), ret);
      close_file_();
    }
  }
  buffer_write_cursor_ = 0;
  return RERCORDER_SUCCESS;
}

inline void InnoPcNpyRecorder::add_cpoint_(void *ctx,
                                           const InnoDataPacket &pkt,
                                           const InnoBlock &block,
                                           const InnoChannelPoint &pt,
                                           const InnoBlockFullAngles &fa,
                                           const uint16_t ch,
                                           const uint16_t m) {
  if (pt.radius == 0) {
    return;
  }
  size_t next_size = buffer_write_cursor_ + sizeof(InnoPcNpy);
  if (next_size > sizeof(pt_buffer_)) {
    flush_buffer_();
  }
  if (size_limit_ > 0) {
    if (total_size_ + ssize_t(sizeof(InnoPcNpy)) > size_limit_) {
      flush_buffer_();
      close_file_();
      return;
    }
  }
  InnoPcNpy *record =
      reinterpret_cast<InnoPcNpy *>(&pt_buffer_[buffer_write_cursor_]);
  record->frame_id = pkt.idx - frame_id_base_;
  record->ts_10us = current_block_ts_10us_offset_ +
                    block.header.ts_10us;
  record->h_angle_unit = fa.angles[ch].h_angle;
  record->v_angle_unit = fa.angles[ch].v_angle;
  record->radius_unit_lo16 = pt.radius & ((1 << 16) - 1);
  record->radius_unit_hi1_refl = pt.refl | ((pt.radius & (1 << 16)) >> 1);
  record->scanline_id = block.header.scan_id;
  record->idx_within_scanline = block.header.scan_idx;
  record->flags_1 = ch;
  record->flags_1 |= (block.header.in_roi == 3) ? (1 << 2) : 0;
  record->flags_1 |= m ? (1 << 3) : 0;
  record->flags_1 |= block.header.facet << 4;
  record->flags_1 |= pkt.scanner_direction << 7;
  record->flags_2 = pkt.confidence_level;
  record->flags_2 |= pt.type << 2;
  record->flags_2 |= pt.elongation << 4;

  total_points_++;
  buffer_write_cursor_ = next_size;
  total_size_ += sizeof(InnoPcNpy);
  return;
}

int InnoPcNpyRecorder::add_block(const InnoDataPacket *pkt) {
  if (!is_opened()) {
    return RERCORDER_FILE_IS_NOT_OPEN;
  }
  if (pkt->type != INNO_ITEM_TYPE_SPHERE_POINTCLOUD &&
      pkt->type != INNO_ITEM_TYPE_XYZ_POINTCLOUD ) {
    return RERCORDER_TYPE_ERROR;
  }
  if (pkt->idx > 1000000000L) {
    // xxx todo: ask gang to fix
    return RERCORDER_INDEX_ERROR;
  }

  if (!record_npy_) {
    if (is_opened()) {
      if (size_limit_ > 0) {
        if (total_size_ + pkt->common.size > size_limit_) {
          close_file_();
          return RERCORDER_SUCCESS;
        }
      }
      int ret = write(fd_, pkt, pkt->common.size);
      if (ret < ssize_t(pkt->common.size)) {
        inno_log_error_errno("write %s failed %d",
                             filename_.c_str(), ret);
        close_file_();
        return RERCORDER_STREAM_ERROR;
      } else {
        total_size_ += pkt->common.size;
      }
    }
    return RERCORDER_SUCCESS;
  }

  size_t ts_10us = pkt->common.ts_start_us / 10;

  if (add_block_called_ == 0) {
    // first call
    frame_id_base_ = pkt->idx;
    timestamp_base_10us_ = ts_10us;
    last_frame_id_ = pkt->idx;
    total_frame_ = 1;
    add_header_(frame_id_base_, ts_10us);
  }
  add_block_called_++;
  if (last_frame_id_ != pkt->idx) {
    // new frame
    last_frame_id_ = pkt->idx;
    total_frame_++;
  }

  size_t pcount = 0;
  current_block_ts_10us_offset_ = ts_10us - timestamp_base_10us_;

  ITERARATE_INNO_DATA_PACKET_CPOINTS(add_cpoint_, NULL, pkt, pcount);

  flush_buffer_();
  return RERCORDER_SUCCESS;
}

void InnoPcNpyRecorder::add_header_(size_t frame_id_base,
                                    size_t timestamp_base_10us) {
  if (!is_opened()) {
    return;
  }
  static const uint32_t kHeaderSize = 1024;
  static const char *version = "1.0";
  char buffer[kHeaderSize];
  memset(buffer, 0, sizeof(buffer));
  int ret = snprintf(
      buffer,
      sizeof(buffer),
      "{\"inno_header_size\": %lu, \n"
      " \"inno_pc_records_version\": \"%s\", \n"
      " \"inno_angle_unit_per_PI_Rad\": %u, \n"
      " \"inno_distance_unit_per_Meter\": %u, \n"
       " \"inno_frame_id_base\": %lu, \n"
      " \"inno_timestamp_base_10us\": %lu, \n"
      " \"inno_point_dtype\": \""
      "frame_id=<u2,"
      "timestamp_10us=<u4,"
      "h_angle_unit=<i2,"
      "v_angle_unit=<i2,"
      "radius_unit=<u2,"
      "refl=<u2,"
      "scanline_id=<u2,"
      "idx_within_scanline=<u2,"
      "flags_1=<u1,"
      "flags_2=<u1\", \n"
      " \"comment_flags_1\": "
      " \"direction[7:7], facet[4:6], return[3:3], roi[2:2], channel[0:1]\", \n"
      " \"comment_flags_2\": "
      " \"elongation[4:7], type[2:3], confidence_level[0:1]\"\n"
      "}\n",
      sizeof(buffer), version,
      kInnoAngleUnitPerPiRad, kInnoDistanceUnitPerMeter,
      frame_id_base, timestamp_base_10us);
  inno_log_verify(ret < ssize_t(sizeof(buffer)), "buffer too small %d", ret);
  ret = write(fd_, buffer, sizeof(buffer));
  if (ret != ssize_t(sizeof(buffer))) {
    inno_log_error_errno("write %s failed %d",
                         filename_.c_str(), ret);
    close(fd_);
    fd_ = -1;
    return;
  } else {
    total_size_ += sizeof(buffer);
  }
  return;
}

}  // namespace innovusion
