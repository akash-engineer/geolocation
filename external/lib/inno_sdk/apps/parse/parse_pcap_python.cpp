/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <string>
#include "sdk_common/converter/cframe_legacy.h"
#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_packet_utils.h"
#include "utils/inno_lidar_log.h"
#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"


static const double kUsInSecond = 1000000.0;
static const double k10UsInSecond = 100000.0;
ssize_t current_cframe_id_ = -1;
ssize_t expect_frame_ = 0;
uint16_t current_sub_frame_;
uint16_t expect_sub_frame_;
double current_cframe_ts_ = 0.0;
ssize_t current_cframe_items = 0;
int64_t frame_so_far_ = -1;
uint64_t data_counter = 0;
uint64_t error_data_counter = 0;
uint64_t frame_counter = 0;
uint64_t miss_frame_counter = 0;
uint64_t miss_sub_frame_times = 0;
uint64_t message_counter = 0;
uint64_t error_message_counter = 0;
uint64_t status_counter = 0;
uint64_t error_status_counter = 0;
uint64_t total_receive_counter = 0;
uint64_t total_successfully_counter = 0;
uint64_t total_failed_counter = 0;
bool next_new_frame_ = false;
inno_cframe_header *current_cframe_;
InnoStatusPacket current_status;
int inno_type = 0;
Eigen::MatrixXd pcd = Eigen::MatrixXd::Zero(1024*500, 6);

enum InnoUDPPackage{
  INNO_NONE = 0,
  INNO_DATA_PACKAGE = 1,
  INNO_MESSAGE_PACKAGE = 2,
  INNO_STATUS_PACKAGE = 3,
};

void init() {
  current_cframe_id_ = -1;
  current_cframe_ts_ = 0;
  current_cframe_items = 0;
  current_sub_frame_ = 0;
  expect_sub_frame_ = 0;
  expect_frame_ = 0;
  frame_so_far_ = -1;
  next_new_frame_ = false;
  data_counter = 0;
  error_data_counter = 0;
  frame_counter = 0;
  miss_frame_counter = 0;
  miss_sub_frame_times = 0;
  message_counter = 0;
  error_message_counter = 0;
  status_counter = 0;
  error_status_counter = 0;
  total_receive_counter = 0;
  total_successfully_counter = 0;
  total_failed_counter = 0;
  current_cframe_ = NULL;
  memset(&current_status, 0, sizeof(current_status));
}

Eigen::MatrixXd parse_data(char *data, int len) {
  Eigen::MatrixXd pcd_group = Eigen::MatrixXd::Zero(1, 1);
  uint32_t return_number;
  uint32_t unit_size;
  InnoDataPacket *pkt = reinterpret_cast<InnoDataPacket *>(data);
  data_counter++;
  // sanity check after relase-2.0.0-rc138
  if (!InnoDataPacketUtils::check_data_packet(*pkt, 0)) {
    inno_log_error("corrupted pkt->idx = %lu", pkt->idx);
    error_data_counter++;
    total_failed_counter++;
    return Eigen::MatrixXd::Zero(1, 1);
  }
  current_sub_frame_ = pkt->sub_idx;
  if (frame_so_far_ >= 0 && expect_sub_frame_ != current_sub_frame_) {
    inno_log_error("UDP sub frame the current_cframe_id_ is %lu"
                  "expect_sub_frame = %u, but current_sub_frame = %u",
                  current_cframe_id_, expect_sub_frame_, current_sub_frame_);
    miss_sub_frame_times++;
  }
  expect_sub_frame_ = current_sub_frame_ + 1;
  if (next_new_frame_) {
    if (current_cframe_id_ + 1 != (int64_t)(pkt->idx)) {
      inno_log_warning("UDP frame the expect is %lu"
                      "but the receive frame is %lu",
                       current_cframe_id_ + 1, pkt->idx);
    } else {
      next_new_frame_ = false;
    }
  }
  if (pkt->is_last_sub_frame) {
    next_new_frame_ = true;
    expect_sub_frame_ = 0;
  }
  InnoDataPacketUtils::get_block_size_and_number_return(*pkt,
                                                          &unit_size,
                                                          &return_number);
  if (pkt->item_number * unit_size + sizeof(InnoDataPacket) != (uint32_t)len) {
    inno_log_error("------pkt ckeck err! expect---- %ld get %d\n",
            pkt->item_number * unit_size + sizeof(InnoDataPacket), len);
    error_data_counter++;
    total_failed_counter++;
    return Eigen::MatrixXd::Zero(1, 1);
  }
  total_successfully_counter++;
  double frame_timestamp_sec = pkt->common.ts_start_us / kUsInSecond;
  const InnoBlock *block =
      reinterpret_cast<const InnoBlock *>(&pkt->inno_block1s[0]);
  if (frame_so_far_ == -1 || ssize_t(pkt->idx) != current_cframe_id_) {
    frame_counter++;
    if (frame_so_far_ > 0) {
      if (expect_frame_ != ssize_t(pkt->idx)) {
        inno_log_warning("The expect frame is %lu,"
                          "but the current frame is %ld",
                            expect_frame_, ssize_t(pkt->idx));
        miss_frame_counter += (ssize_t(pkt->idx) - expect_frame_);
      }
      pcd.conservativeResize(current_cframe_items, 6);
      current_cframe_items = 0;
      pcd_group = pcd;
      pcd.resize(1024*500, 6);
    }
    current_cframe_id_ = ssize_t(pkt->idx);
    expect_frame_ = current_cframe_id_ + 1;
    current_cframe_ts_ = pkt->common.ts_start_us;
    frame_so_far_++;
  }
  // we want to skip the first frame, which may be partial
  if (frame_so_far_ > 0) {
    for (uint32_t i  = 0; i < pkt->item_number;
          i++, block = reinterpret_cast<const InnoBlock *>
                      (reinterpret_cast<const char *>(block) + unit_size)) {
      // calculate (x,y,z) cartesian coordinate from spherical coordinate
      // for each point in the block
      // 1. use get_full_angles() to restore angle for each channel
      // 2. use get_xyzr_meter() to calculate (x,y,z)
      InnoBlockFullAngles full_angles;
      InnoDataPacketUtils::get_block_full_angles(&full_angles, block->header);
      for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
        for (uint32_t m = 0; m < return_number; m++) {
          const InnoChannelPoint &pt =
              block->points[InnoBlock2::get_idx(channel, m)];
          InnoXyzrD xyzr;
          if (pt.radius > 0) {
            InnoDataPacketUtils::get_xyzr_meter(
                full_angles.angles[channel],
                pt.radius, channel,
                &xyzr);
            pcd(current_cframe_items, 0) = xyzr.x;
            pcd(current_cframe_items, 1) = xyzr.y;
            pcd(current_cframe_items, 2) = xyzr.z;
            pcd(current_cframe_items, 3) = pt.refl;
            pcd(current_cframe_items, 4) = frame_timestamp_sec +
                           block->header.ts_10us / k10UsInSecond;
            pcd(current_cframe_items, 5) =
                         ((uint16_t)(block->header.scan_id) << 8) + channel +
                         ((pkt->multi_return_mode & 0x1) << 2) +
                         (uint16_t)(block->header.in_roi << 3);
            current_cframe_items++;
          }
        }
      }
    }
  }
  if (pcd_group.rows() > 1) {
    return pcd_group;
  } else  {
    return Eigen::MatrixXd::Zero(1, 1);
  }
}

bool parse_status(char *data, int len) {
  InnoStatusPacket *pkt = reinterpret_cast<InnoStatusPacket *>(data);
  current_cframe_ts_ = pkt->common.ts_start_us;
  status_counter++;
  // sanity check after relase-2.0.0-rc138
  if (!InnoDataPacketUtils::check_status_packet(*pkt, 0)) {
    inno_log_warning("corrupted pkt->idx = %lu", pkt->idx);
    error_status_counter++;
    return false;
  }
  return true;
}

bool parse_message(char *data, int len) {
  InnoDataPacket *pkt = reinterpret_cast<InnoDataPacket *>(data);
  message_counter++;
  // sanity check after relase-2.0.0-rc138
  if (!InnoDataPacketUtils::check_data_packet(*pkt, 0)) {
    inno_log_warning("corrupted pkt->idx = %lu", pkt->idx);
    error_message_counter++;
    return false;
  }
  InnoMessage *msg = &pkt->messages[0];
  uint32_t level = msg->level;
  uint32_t code = msg->code;
  inno_log_info("level = %u code = %u\n", level, code);
  switch (level) {
    case INNO_MESSAGE_LEVEL_INFO:
      inno_log_info("content = %s\n", msg->content);
      break;
    case INNO_MESSAGE_LEVEL_WARNING:
      inno_log_warning("content = %s\n", msg->content);
      break;
    case INNO_MESSAGE_LEVEL_ERROR:
      inno_log_error("content = %s\n", msg->content);
      break;
    case INNO_MESSAGE_LEVEL_FATAL:
      inno_log_fatal("content = %s\n", msg->content);
      break;
    default:
      inno_log_info("content = %s\n", msg->content);
      break;
  }
  current_cframe_ts_ = pkt->common.ts_start_us;
  return true;
}

Eigen::MatrixXd parse_inno_package(char *data, int len, int debug_index) {
  bool ret = false;
  total_receive_counter++;
  // inno_log_info("index = %d  %d\n", debug_index, len);
  InnoStatusPacket *statusPkt = reinterpret_cast<InnoStatusPacket *>(data);
  if (statusPkt->common.version.magic_number == kInnoMagicNumberStatusPacket) {
    inno_type = INNO_STATUS_PACKAGE;
    ret = parse_status(data, len);
  } else if (statusPkt->common.version.magic_number
                     == kInnoMagicNumberDataPacket) {
    InnoDataPacket *pkt = reinterpret_cast<InnoDataPacket *>(data);
    if (pkt->type == INNO_ITEM_TYPE_MESSAGE ||
        pkt->type == INNO_ITEM_TYPE_MESSAGE_LOG ) {
      ret = parse_message(data, len);
      inno_type = INNO_MESSAGE_PACKAGE;
    } else if (pkt->type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD ||
               pkt->type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
      inno_type = INNO_DATA_PACKAGE;
      return parse_data(data, len);
    } else {
      inno_type = INNO_NONE;
      inno_log_warning("type error\n");
    }
  } else {
    inno_type = INNO_NONE;
    inno_log_warning("magic numbe error\n");
  }
  if (ret) {
    total_successfully_counter++;
  } else {
    total_failed_counter++;
  }
  return Eigen::MatrixXd::Zero(1, 1);
}

void summary_print() {
  inno_log_info("----------Summary----------\n"
              "data_counter = %lu, error_data_counter = %lu\n"
              "frame_counter = %lu, miss_frame_counter = %lu, "
                                "miss_sub_frame_times = %lu\n"
              "message_counter = %lu, error_message_counter = %lu\n"
              "status_counter = %lu, error_status_counter = %lu\n"
              "total_receive_counter = %lu\n"
              "total_successfully_counter = %lu,"
              "total_failed_counter = %lu\n",
              data_counter, error_data_counter,
              frame_counter, miss_frame_counter, miss_sub_frame_times,
              message_counter, error_message_counter,
              status_counter, error_status_counter,
              total_receive_counter,
              total_successfully_counter, total_failed_counter);
}


PYBIND11_MODULE(inno_falcon_b_parse, m) {
  m.def("init", &init);
  m.def("parse_inno_package", &parse_inno_package);
  m.def("summary_print", &summary_print);
  m.def("idx", []() { return current_cframe_id_ - 1; });
  m.def("ts_us_start", []() { return current_cframe_ts_; });
  m.def("inno_type", []() { return inno_type; });
  m.def("status_idx", []()
                 { return current_status.idx; });
  m.def("in_faults", []() { return current_status.in_faults.faults; });
  m.def("ex_faults", []() { return current_status.ex_faults.faults; });
}
