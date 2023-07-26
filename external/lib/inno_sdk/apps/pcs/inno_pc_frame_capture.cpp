/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/inno_pc_frame_capture.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "src/sdk_common/converter/png_recorder.h"
#include "src/sdk_common/converter/rosbag_recorder.h"
#include "src/sdk_common/inno_lidar_api.h"
#include "src/sdk_common/inno_lidar_packet_utils.h"
#include "src/utils/inno_lidar_log.h"
#include "src/utils/mem_pool_manager.h"
#include "src/utils/net_manager.h"
#include "src/utils/utils.h"
#include "pcs/pc_server_ws_processor.h"
#include "pcs/pcs.h"

namespace innovusion {
const char *InnoPcFrameCapture::pcd_header_ =
    "# .PCD v.7 - Point Cloud Data file format\n"
    "FIELDS "
    "x y z %s channel roi facet is_2nd_return multi_return confid_level "
    "flag elongation timestamp "
    "scanline scan_idx frame_id\n"
    "SIZE 4 4 4 2 1 1 1 1 1 1 1 1 8 2 2 4\n"
    "TYPE F F F U U U U U U U U U F U U U\n"
    "COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n"
    "WIDTH %lu\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS %lu\n"
    "DATA %s\n";
const char *InnoPcFrameCapture::pcd_row_ = "%.3f %.3f %.3f %u %u "
                           "%u %u %u %u %u %u %u %.5f %u %u %lu\n";

const char *InnoPcFrameCapture::csv_header_ =
    "x,y,z,%s,channel,roi,facet,is_2nd_return,multi_return,confid_level,"
    "flag,elongation,timestamp,"
    "scanline,scan_idx,frame_id\n";
const char *InnoPcFrameCapture::csv_row_ = "%.3f,%.3f,%.3f,%u,%u,"
                           "%u,%u,%u,%u,%u,%u,%u,%.5f,%u,%u,%lu\n";

InnoPcFrameCapture::InnoPcFrameCapture(PCS *pcs) {
  pcs_ = pcs;
  reflectance_ = true;
  worker_ = NULL;
  rosbag_stream_ = NULL;
  png_stream_ = NULL;
  pc_packet_pool_ = nullptr;

  {
    std::unique_lock<std::mutex> lk(mutex_);
    reset_job_state_with_lock_();
  }

  size_t pcd_len_saved =
      sizeof(InnoDataPacket) + sizeof(InnoBlock2) * kMaxBlockNum;
  pcd_saved_packet_ =
      reinterpret_cast<InnoDataPacket *>(calloc(pcd_len_saved, 1));
  inno_log_verify(pcd_saved_packet_, "cannot allocate saved_ %lu",
                  pcd_len_saved);

  size_t pcd_header_len_saved = sizeof(InnoDataPacket) * kMaxPacketNum;
  pcd_saved_packet_headers_ =
      reinterpret_cast<InnoDataPacket *>(calloc(pcd_header_len_saved, 1));
  inno_log_verify(pcd_saved_packet_headers_, "cannot allocate saved_header %lu",
                  pcd_header_len_saved);
}

InnoPcFrameCapture::~InnoPcFrameCapture() {
  // join thread
  {
    std::unique_lock<std::mutex> lk(mutex_);
    while (worker_ != NULL) {
      pc_shutdown_ = true;
      pc_cond_.notify_all();
      pc_cond_.wait(lk, [this] { return worker_ == NULL;});
    }

    while (!pc_packet_deque_.empty()) {
      InnoDataPacket *q = pc_packet_deque_[0];
      pc_packet_deque_.pop_front();
      if (q) {
        pc_packet_pool_->free(q);
      }
    }
    end_job_with_lock_();
  }
  if (pcd_saved_packet_) {
    free(pcd_saved_packet_);
  }
  if (pcd_saved_packet_headers_) {
    free(pcd_saved_packet_headers_);
  }
  if (pc_packet_pool_) {
    delete pc_packet_pool_;
    pc_packet_pool_ = NULL;
  }
}

void InnoPcFrameCapture::reset_job_state_with_lock_() {
  job_type_ = CAPTURE_TYPE_NONE;
  job_conn_ = NULL;
  job_duration_ = 0;
  display_job_duration_ = 0;
  pcd_block_so_far_ = 0;
  pcd_packet_so_far_ = 0;
  pcd_start_frame_ = -1;
  pcd_last_frame_ = -1;
  pcd_frame_captured_ = 0;
  pc_shutdown_ = false;
  null_enqueued_ = false;
  raw2_sent_bytes_ = 0;
  raw2_callback_ = 0;
  current_point_in_frame_ = 0;
  bag_stream_status_ = BAG_STREAM_INIT;
  inno_log_verify(worker_ == NULL, "worker_");
  inno_log_verify(rosbag_stream_ == NULL, "rosbag_stream");
  inno_log_verify(png_stream_ == NULL, "png_stream");

  clear_pc_packet_deque_w_lock_();
}

int InnoPcFrameCapture::add_capture_job(const std::string &type,
                                        const std::string &duration,
                                        void *ctx) {
  {
    std::unique_lock<std::mutex> lk(mutex_);
    if (job_type_ != CAPTURE_TYPE_NONE) {
      inno_log_info("capture is busy");
      return 503;
    }
    reset_job_state_with_lock_();
    job_duration_ = atoi(duration.c_str());
    if (type == "pcd" || type == "pcd_binary" || type == "csv") {
      if (type == "csv") {
        job_type_ = CAPTURE_TYPE_CSV;
      } else {
        job_type_ = type == "pcd" ?
                    CAPTURE_TYPE_PCD :
                    CAPTURE_TYPE_PCD_BINARY;
      }
      if (job_duration_ < 1) {
        job_duration_ = 1;
      }
      if (job_duration_ > 20) {
        job_duration_ = 20;
      }
      inno_log_info("add capture pcd job duration=%lu", job_duration_);
      display_job_duration_ = job_duration_;
      job_conn_ = *(std::shared_ptr<WsConnection> *)(ctx);
    } else if (type == "pc" || type == "inno_pc" || type == "inno_pc_xyz") {
      if (type == "inno_pc_xyz") {
        job_type_ = CAPTURE_TYPE_PC_XYZ;
      } else {
        job_type_ = CAPTURE_TYPE_PC;
      }
      if (job_duration_ < 1) {
        job_duration_ = 1;
      }
      display_job_duration_ = job_duration_;
      inno_log_info("add capture %s job duration=%lu",
                    type.c_str(), job_duration_);
      job_conn_ = *(std::shared_ptr<WsConnection> *)(ctx);
    } else if (type == "bag") {
      job_type_ = CAPTURE_TYPE_BAG;
      if (job_duration_ < 1) {
        job_duration_ = 1;
      }
      display_job_duration_ = job_duration_;
      inno_log_info("add capture bag job duration=%lu", job_duration_);
      job_conn_ = *(std::shared_ptr<WsConnection> *)(ctx);
      bag_stream_status_ = BAG_STREAM_START;
      rosbag_stream_ = new RosbagRecorder("", write_to_wconn_s_,
                                          this, -1);
      inno_log_verify(rosbag_stream_ != NULL, "can't create RosbagRecorder");
    } else if (type == "png") {
      job_type_ = CAPTURE_TYPE_PNG;
      if (job_duration_ < 1) {
        job_duration_ = 1;
      }
      if (job_duration_ > 20) {
        job_duration_ = 20;
      }
      inno_log_info("add capture png job duration=%lu", job_duration_);
      job_conn_ = *(std::shared_ptr<WsConnection> *)(ctx);

      inno_log_verify(png_stream_ == NULL, "png_stream_ creating");
      png_stream_ = new PngRecorder(job_duration_, kMaxPacketNum,
                                    kMaxBlockNum, true);
      inno_log_verify(png_stream_ != NULL, "can't create PngRecorder");
    } else if (type == "raw" || type == "inno_raw" ||
               type == "raw_raw" || type == "yaml") {
      job_duration_ = atoi(duration.c_str());
      display_job_duration_ = job_duration_;
      if (type == "yaml") {
        job_type_ = CAPTURE_TYPE_YAML;
        display_job_duration_ = -1;
      } else if (type == "raw_raw") {
        raw_raw_channel_ = atoi(duration.c_str());
        if (raw_raw_channel_ < 0 || raw_raw_channel_ >= 4) {
          inno_log_info("invalid capture channel %d",
                        raw_raw_channel_);
          return 400;
        }
        job_type_ = CAPTURE_TYPE_RAW_RAW;
        stop_force_ = false;
      } else {
        job_type_ = CAPTURE_TYPE_RAW;
        if (job_duration_ < 1) {
          job_duration_ = 1;
        }
        display_job_duration_ = job_duration_;
        job_duration_ *= 1024 * 1024;  // in MB
        stop_force_ = false;
      }
      job_conn_ = *(std::shared_ptr<WsConnection> *)(ctx);

      while (1) {
        int ret = pcs_->set_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW2,
                                              recorder_callback_raw2_s_, this);
        if (ret == 0) {
          return 0;
        } else {
          usleep(10 * 1000);
          inno_log_info("retry set recorder");
        }
      }
    } else {
      inno_log_info("invalid capture job %s", type.c_str());
      return 400;
    }
  }
  return 0;
}

void InnoPcFrameCapture::prepare_http_header(char *buffer, size_t buffer_len,
                                             char duration_type,
                                             ssize_t duration,
                                             const char *type,
                                             const char *ext) {
  capture_filename_ = std::string("sn") + pcs_->get_sn() + "_" +
                      InnoUtils::get_current_time_str();
  if (duration >= 0) {
    capture_filename_ += "_";
    capture_filename_ += std::to_string(duration) + duration_type;
  }
  capture_filename_ += std::string(".") + ext;

  int ret = snprintf(buffer, buffer_len,
                     PcServerWsProcessor::reply_capture_template,
                     type, capture_filename_.c_str());
  inno_log_info("catpure filename: %s", capture_filename_.c_str());
  inno_log_verify(ret < ssize_t(buffer_len), "too small %d", ret);
  return;
}

int InnoPcFrameCapture::recorder_callback_raw2_(
    enum InnoRecorderCallbackType type, const char *buffer, int len) {
  static const uint32_t kYamlWriteCallback = 3;
  int ret;
  std::unique_lock<std::mutex> lk(mutex_);

  inno_log_verify(CAPTURE_TYPE_RAW == job_type_ ||
                  CAPTURE_TYPE_YAML == job_type_ ||
                  CAPTURE_TYPE_RAW_RAW == job_type_,
                  "job_type=%d", job_type_);

  if (raw2_callback_ == 0) {
    // send header
    char http[1024];
    prepare_http_header(http, sizeof(http),
                        job_type_ == CAPTURE_TYPE_RAW_RAW ?
                        'C' : 'M',
                        display_job_duration_,
                        "application/octet-stream",
                        job_type_ == CAPTURE_TYPE_RAW ?
                        "inno_raw" :
                        (job_type_ == CAPTURE_TYPE_RAW_RAW ?
                         "inno_raw_raw" :
                         "yaml"));

    lk.unlock();
    ret = PcServerWsProcessor::write_buffer_to_ws_socket_full_s(
        &job_conn_, http,
        strlen(http));
    lk.lock();

    if (ret < 0) {
      end_job_with_lock_();
      return ret;
    }
  }

  raw2_callback_++;

  bool need_send = false;
  bool ignore_and_return = false;
  bool end_capture = false;

  if (len > 0) {
    if (job_type_ == CAPTURE_TYPE_RAW) {
      if (len + raw2_sent_bytes_ <= job_duration_ && !stop_force_) {
        need_send = true;
      } else {
        end_capture = true;
      }
    } else if (job_type_ == CAPTURE_TYPE_RAW_RAW) {
      if (raw2_callback_ <= kYamlWriteCallback) {
        need_send = true;
      } else {
        inno_log_info("yaml captured, start capture raw_raw");
        inno_log_verify(worker_ == NULL, "worker_ not null");
        worker_ = new std::thread([this]() {
                                    send_raw_raw_capture_thread_();
                                  });
        inno_log_verify(worker_, "worker");
        return -1;
      }
    } else if (job_type_ == CAPTURE_TYPE_YAML) {
      if (raw2_callback_ == kYamlWriteCallback) {
        int real_len = 0;
        while (real_len < len) {
          if (buffer[real_len] == 0) {
            len = real_len;
            break;
          }
          real_len++;
        }
        need_send = true;
      } else if (raw2_callback_ <= kYamlWriteCallback) {
        ignore_and_return = true;
      } else {
        end_capture = true;
      }
    } else {
      inno_log_panic("impossible %d", job_type_);
    }
  } else {
    end_capture = true;
  }

  if (need_send) {
    lk.unlock();
    ret = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
        &job_conn_, buffer,
        len);
    lk.lock();
    if (ret < 0) {
      end_job_with_lock_();
      return ret;
    } else {
      raw2_sent_bytes_ += len;
      return 0;
    }
  } else if (ignore_and_return) {
    return 0;
  } else if (end_capture) {
    lk.unlock();
    PcServerWsProcessor::write_chunk_to_ws_socket_full_s(&job_conn_,
                                                               NULL, 0);
    lk.lock();
    end_job_with_lock_();
    inno_log_info("inno_raw capture %s done sending %lu bytes.",
                  capture_filename_.c_str(), raw2_sent_bytes_);
    return -1;
  } else {
    inno_log_panic("impossible");
    return 0;
  }
}

void InnoPcFrameCapture::send_raw_raw_capture_thread_() {
  int ret = 0;
  do {
    pcs_->pause_lidar();
    ret = pcs_->set_lidar_attribute("raw_channel",
                                    std::to_string(raw_raw_channel_));
    if (ret != 0) {
      inno_log_warning("cannot set raw_raw %d", raw_raw_channel_);
      break;
    }

    std::string lidar_ip(pcs_->get_lidar_ip());
    if (lidar_ip.empty()) {
      inno_log_warning("not live lidar for raw_raw");
      break;
    }
    innovusion::NetManager nm("127.0.0.1",
                              pcs_->get_lidar_port(), 1);
    int len = 0;
    int sockfd = nm.send_command_return_fd(NULL, &len, "start");
    if (sockfd < 0) {
      inno_log_warning("cannot open fd for raw_raw capture");
      break;
    }

    uint64_t total = 0;
    char recvBuff[256 * 1024];

    while (1) {
      {
        std::unique_lock<std::mutex> lk(mutex_);
        if (stop_force_) {
          inno_log_info("stop raw_raw capturing.");
          break;
        }
      }
      ssize_t r = nm.recv_full_buffer(sockfd, recvBuff, sizeof(recvBuff), 0);
      if (r <= 0) {
        inno_log_warning("raw_raw recv failed %ld", r);
        break;
      }

      if (r != sizeof(recvBuff)) {
        inno_log_warning("error in reading raw raw %ld != %lu",
                         r, sizeof(recvBuff));
        break;
      }

      ret = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
          &job_conn_,
          recvBuff,
          r);

      if (ret < 0) {
        inno_log_warning("error in writing raw_raw %d %ld",
                         ret, r);
        break;
      }

      total += r;
      size_t kMaxRawRawCaptureSize = 256 * 1024 * 1024;
      if (total >= kMaxRawRawCaptureSize) {
        inno_log_info("raw_raw captured %lu", total);
        break;
      }
    }
    close(sockfd);
  } while (0);

  pcs_->set_lidar_attribute("raw_channel", "-1");
  pcs_->resume_lidar();

  ret = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(&job_conn_,
                                                            NULL, 0);
  if (ret < 0) {
    inno_log_warning("error in writing raw_raw %d",
                     ret);
  } else {
    ret = PcServerWsProcessor::flush_buffer_s(&job_conn_);
    if (ret < 0) {
      inno_log_warning("error in finalize raw_raw %d",
                       ret);
    } else {
      inno_log_info("done raw_raw captured");
    }
  }

  {
    std::unique_lock<std::mutex> lk(mutex_);
    end_worker_with_lock_();
  }
}

void InnoPcFrameCapture::received_data_packet(const InnoDataPacket *pkt) {
  if (pkt->type != INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
    return;
  }
  // quick check without lock
  if (job_type_ == CAPTURE_TYPE_PCD ||
      job_type_ == CAPTURE_TYPE_PCD_BINARY ||
      job_type_ == CAPTURE_TYPE_CSV) {
    pcd_received_data_packet_(pkt);
  } else if (job_type_ == CAPTURE_TYPE_PC ||
             job_type_ == CAPTURE_TYPE_PC_XYZ ||
             job_type_ == CAPTURE_TYPE_BAG) {
    if (!pc_packet_pool_) {
      pc_packet_pool_ =
          new MemPool("pc_packet_pool", kMaxPacketSize, kPacketPoolSize, 32);
      inno_log_verify(pc_packet_pool_, "pc_packet_pool_");
    }
    pc_received_data_packet_(pkt);
  } else if (job_type_ == CAPTURE_TYPE_PNG) {
    png_received_data_packet_(pkt);
  } else {
    return;
  }
}

void InnoPcFrameCapture::png_received_data_packet_(const InnoDataPacket *pkt) {
  std::unique_lock<std::mutex> lk(mutex_);
  if (job_type_ != CAPTURE_TYPE_PNG) {
    return;
  }

  inno_log_verify(png_stream_ != NULL, "png_stream_");

  if (!worker_) {
    bool is_saving = png_stream_->capture(pkt);
    if (is_saving) {
      send_png_capture_thread_();
    }
  }
}

void InnoPcFrameCapture::pcd_received_data_packet_(const InnoDataPacket *pkt) {
  {
    std::unique_lock<std::mutex> lk(mutex_);
    if (job_type_ != CAPTURE_TYPE_PCD &&
        job_type_ != CAPTURE_TYPE_PCD_BINARY &&
        job_type_ != CAPTURE_TYPE_CSV) {
      return;
    }
  }

  if (pcd_start_frame_ < 0) {
    // haven't find
    if (pcd_last_frame_ >= 0 && pcd_last_frame_ != ssize_t(pkt->idx)) {
      // first subframe in new frame
      inno_log_info("capture first frame %lu", pkt->idx);

      pcd_start_frame_ = pkt->idx;
      current_point_in_frame_ = 0;
      reflectance_ = pkt->use_reflectance;
      pcd_add_data_packet_(pkt);
    }
  } else {
    inno_log_verify(pcd_last_frame_ >= 0, "pcd_last_frame_ %ld",
                    pcd_last_frame_);
    if (pcd_last_frame_ != ssize_t(pkt->idx)) {
      pcd_frame_captured_++;
      if (pcd_frame_captured_ <= job_duration_) {
        inno_log_info("capture %u points in frame-%lu-%ld to pcd",
                      current_point_in_frame_, pcd_frame_captured_,
                      pcd_last_frame_);
      }
      current_point_in_frame_ = 0;
    }
    if (pcd_frame_captured_ + 1 <= job_duration_) {
      pcd_add_data_packet_(pkt);
    } else {
      std::unique_lock<std::mutex> lk(mutex_);
      if ((worker_ == NULL) && (job_type_ != CAPTURE_TYPE_NONE)) {
        inno_log_info("capture pc frame done, sending...");
        inno_log_verify(worker_ == NULL, "worker_");
        worker_ = new std::thread([this]() {
          InnoUtils::set_self_thread_priority(-20);
          send_pcd_capture_thread_();
        });
        inno_log_verify(worker_, "worker");
      }
    }
  }
  pcd_last_frame_ = pkt->idx;
}

void InnoPcFrameCapture::pc_received_data_packet_(const InnoDataPacket *pkt) {
  {
    std::unique_lock<std::mutex> lk(mutex_);
    if (job_type_ != CAPTURE_TYPE_PC &&
        job_type_ != CAPTURE_TYPE_PC_XYZ &&
        job_type_ != CAPTURE_TYPE_BAG) {
      return;
    }
  }
  bool enque_pkt = false;
  if (pcd_start_frame_ < 0) {
    if (pcd_last_frame_ >= 0 && pcd_last_frame_ != ssize_t(pkt->idx)) {
      // first subframe in new frame
      inno_log_info("capture first frame %lu", pkt->idx);

      pcd_start_frame_ = pkt->idx;
      pcd_last_frame_ = pkt->idx;
      // xxx todo start thread
      worker_ = new std::thread([this]() {
        InnoUtils::set_self_thread_priority(-20);
        if (job_type_ == CAPTURE_TYPE_BAG) {
          send_bag_capture_thread_();
        } else {
          send_inno_pc_capture_thread_();
        }
      });
      enque_pkt = true;
    }
  } else {
    inno_log_verify(pcd_last_frame_ >= 0, "pcd_last_frame_ %ld",
                    pcd_last_frame_);
    if (pcd_last_frame_ != ssize_t(pkt->idx)) {
      pcd_frame_captured_++;
    }
    if (pcd_frame_captured_ + 1 <= job_duration_) {
      if (pcd_last_frame_ != ssize_t(pkt->idx)) {
        inno_log_trace("capture frame %lu", pkt->idx);
      }
      enque_pkt = true;
    } else {
      if (!null_enqueued_) {
        inno_log_info("stop capturing, last=%ld %lu",
                      pcd_last_frame_, pkt->idx);
        std::unique_lock<std::mutex> lk(mutex_);
        pc_packet_deque_.push_back(NULL);
        null_enqueued_ = true;
      }
    }
  }
  pcd_last_frame_ = pkt->idx;

  if (enque_pkt) {
    InnoDataPacket *p =
        reinterpret_cast<InnoDataPacket *>(pc_packet_pool_->alloc());
    if (p == NULL) {
      inno_log_warning("run out of buffer, stop capturing");
      {
        std::unique_lock<std::mutex> lk(mutex_);
        pc_shutdown_ = true;
      }
    } else {
      inno_log_verify(pkt->common.size <= kMaxPacketSize,
                      "pkt size too big %u",
                      pkt->common.size);
      memcpy(p, pkt, pkt->common.size);
      {
        std::unique_lock<std::mutex> lk(mutex_);
        pc_packet_deque_.push_back(p);
      }
    }
  }
  pc_cond_.notify_all();
  return;
}

void InnoPcFrameCapture::clear_pc_packet_deque_w_lock_() {
  while (!pc_packet_deque_.empty()) {
    InnoDataPacket *q = pc_packet_deque_[0];
    pc_packet_deque_.pop_front();
    if (q) {
      inno_log_verify(pc_packet_pool_, "pc_packet_pool_");
      pc_packet_pool_->free(q);
    }
  }
}

void InnoPcFrameCapture::pcd_add_data_packet_(const InnoDataPacket *pkt) {
  inno_log_verify(pkt, "pkt");
  {
    std::unique_lock<std::mutex> lk(mutex_);
    inno_log_verify(job_type_ == CAPTURE_TYPE_PCD ||
                    job_type_ == CAPTURE_TYPE_PCD_BINARY ||
                    job_type_ == CAPTURE_TYPE_CSV,
                    "job_type_ %d", job_type_);
    inno_log_verify(pcd_last_frame_ >= 0, "pcd_last_frame_ %ld",
                    pcd_last_frame_);
  }

  if (pcd_packet_so_far_ >= kMaxPacketNum) {
    inno_log_warning("reach limit kMaxPacketNum %u", pcd_packet_so_far_);
    return;
  }

  uint32_t block_left = kMaxBlockNum - pcd_block_so_far_;
  uint32_t block_to_copy = pkt->item_number;
  if (block_to_copy > block_left) {
    block_to_copy = block_left;
    inno_log_warning("reach limit kMaxBlockNum %lu", kMaxBlockNum);
  }
  current_point_in_frame_ += InnoDataPacketUtils::get_points_count(*pkt);
  for (uint32_t i = 0; i < block_to_copy; i++) {
    if (pkt->multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
      memcpy(pcd_saved_packet_->inno_block2s + pcd_block_so_far_,
             &pkt->inno_block1s[i], sizeof(InnoBlock1));
    } else {
      memcpy(pcd_saved_packet_->inno_block2s + pcd_block_so_far_,
             &pkt->inno_block2s[i], sizeof(InnoBlock2));
    }
    pcd_block_so_far_++;
  }

  if (block_to_copy > 0) {
    memcpy(pcd_saved_packet_headers_ + pcd_packet_so_far_, pkt,
           sizeof(InnoDataPacket));
    pcd_packet_so_far_++;
  }
  return;
}

void InnoPcFrameCapture::send_pcd_capture_thread_() {
  {
    std::unique_lock<std::mutex> lk(mutex_);
    inno_log_verify(job_type_ == CAPTURE_TYPE_PCD ||
                    job_type_ == CAPTURE_TYPE_PCD_BINARY ||
                    job_type_ == CAPTURE_TYPE_CSV,
                    "job_type_ %d", job_type_);
    inno_log_verify(pcd_last_frame_ >= 0, "pcd_last_frame_ %ld",
                    pcd_last_frame_);
  }
  int ret;
  const size_t kMaxChunkLen = 65536;
  const size_t kMaxOneLineLen = 1024;
  char buffer[kMaxChunkLen];
  size_t total_sent = 0;

  // send header
  if (job_type_ == CAPTURE_TYPE_PCD) {
    prepare_http_header(buffer, sizeof(buffer),
                        'F',
                        display_job_duration_,
                        "text/pcd", "pcd");
  } else if (job_type_ == CAPTURE_TYPE_CSV) {
    prepare_http_header(buffer, sizeof(buffer),
                        'F',
                        display_job_duration_,
                        "text/csv", "csv");
  } else {
    prepare_http_header(buffer, sizeof(buffer),
                        'F',
                        display_job_duration_,
                        "application/octet-stream", "pcd");
  }

  int rt;
  rt = PcServerWsProcessor::write_buffer_to_ws_socket_full_s(
      &job_conn_, buffer,
      strlen(buffer));
  if (rt < 0) {
    end_worker_();
    inno_log_info("pcd capture %s failed sending %lu bytes.",
                  capture_filename_.c_str(), total_sent);
    return;
  }

  // send data header
  uint64_t total_points = 0;
  for (size_t i = 0; i < pcd_block_so_far_; i++) {
    for (uint32_t ch = 0; ch < kInnoChannelNumber; ch++) {
      for (uint32_t m = 0; m < 2; m++) {
        InnoChannelPoint &pt = pcd_saved_packet_->inno_block2s[i]
                               .points[InnoBlock2::get_idx(ch, m)];
        if (pt.radius > 0) {
          total_points++;
        }
      }
    }
  }
  inno_log_info(
      "send_capture first_frame=%ld duration=%lu, "
      "blocks=%u, point=%lu",
      pcd_start_frame_, job_duration_, pcd_block_so_far_, total_points);
  if (job_type_ == CAPTURE_TYPE_CSV) {
    ret = snprintf(buffer, sizeof(buffer),
                   InnoPcFrameCapture::csv_header_,
                   reflectance_ ? "reflectance" : "intensity");
  } else {
    ret = snprintf(buffer, sizeof(buffer),
                   InnoPcFrameCapture::pcd_header_,
                   reflectance_ ? "reflectance" : "intensity",
                   total_points, total_points,
                   job_type_ == CAPTURE_TYPE_PCD ?
                   "ascii" : "binary");
  }
  inno_log_verify(ret < ssize_t(sizeof(buffer)), "too small %d", ret);
  rt = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
      &job_conn_, buffer,
      strlen(buffer));
  if (rt < 0) {
    end_worker_();
    inno_log_info("pcd capture %s failed sending %lu bytes.",
                  capture_filename_.c_str(), total_sent);
    return;
  }
  total_sent += ret;

  // send data
  InnoBlock2 *block;
  size_t byte_so_far = 0;
  size_t current_packet = 0;
  size_t bks_sum = 0;
  static const double kUsInSecond = 1000000.0;
  static const double k10UsInSecond = 100000.0;
  const char *row_format;
  if (job_type_ == CAPTURE_TYPE_PCD ||
      job_type_ == CAPTURE_TYPE_CSV) {
    row_format = job_type_ == CAPTURE_TYPE_CSV
                ? InnoPcFrameCapture::csv_row_
                : InnoPcFrameCapture::pcd_row_;
  }
  for (size_t i = 0; i < pcd_block_so_far_; i++) {
    inno_log_verify(current_packet < pcd_packet_so_far_, "%lu vs %u",
                    current_packet, pcd_packet_so_far_);
    block = &pcd_saved_packet_->inno_block2s[i];
    InnoBlockFullAngles full_angles;
    InnoDataPacketUtils::get_block_full_angles(&full_angles, block->header);
    double timestamp_sec =
        pcd_saved_packet_headers_[current_packet].common.ts_start_us /
        kUsInSecond;
    uint32_t confidence_level =
        pcd_saved_packet_headers_[current_packet].confidence_level;
    uint64_t frame_id = pcd_saved_packet_headers_[current_packet].idx;
    for (uint32_t ch = 0; ch < kInnoChannelNumber; ch++) {
      for (uint32_t m = 0; m < 2; m++) {
        InnoChannelPoint &pt = block->points[InnoBlock2::get_idx(ch, m)];
        if (pt.radius > 0) {
          InnoXyzrD xyzr;
          InnoDataPacketUtils::get_xyzr_meter(full_angles.angles[ch], pt.radius,
                                              ch, &xyzr);
          double ptime = timestamp_sec + block->header.ts_10us / k10UsInSecond;
          if (job_type_ == CAPTURE_TYPE_PCD ||
              job_type_ == CAPTURE_TYPE_CSV) {
            ret = snprintf(buffer + byte_so_far,
                           sizeof(buffer) - byte_so_far,
                           row_format,
                           xyzr.x, xyzr.y, xyzr.z, pt.refl, ch,
                           (uint32_t)block->header.in_roi,
                           (uint32_t)block->header.facet, pt.is_2nd_return,
                           m, confidence_level,
                           (uint32_t)pt.type, (uint32_t)pt.elongation,
                           ptime, (uint32_t)block->header.scan_id,
                           (uint32_t)block->header.scan_idx, frame_id);
          } else if (job_type_ == CAPTURE_TYPE_PCD_BINARY) {
            PcdPoint *pct = reinterpret_cast<PcdPoint *>(buffer + byte_so_far);
            pct->x = xyzr.x;
            pct->y = xyzr.y;
            pct->z = xyzr.z;
            pct->intensity = pt.refl;
            pct->channel = ch;
            pct->roi = block->header.in_roi;
            pct->facet = block->header.facet;
            pct->is_2nd_return = pt.is_2nd_return;
            pct->multi_return = m;
            pct->confid_level = confidence_level;
            pct->flags = pt.type;
            pct->elongation = pt.elongation;
            pct->timestamp = ptime;
            pct->scanline = block->header.scan_id,
            pct->scan_idx = block->header.scan_idx;
            pct->frame_id = frame_id;
            ret = sizeof(*pct);
          } else {
            inno_log_panic("invalid type %d", job_type_);
          }
          byte_so_far += ret;
          total_sent += ret;
          inno_log_verify(byte_so_far <= sizeof(buffer), "too small %d", ret);
          if (byte_so_far > sizeof(buffer) - kMaxOneLineLen) {
            rt = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
                &job_conn_, buffer,
                byte_so_far);
            if (rt < 0) {
              end_worker_();
              inno_log_info("pcd capture %s failed sending %lu bytes.",
                            capture_filename_.c_str(), total_sent);
              return;
            }
            byte_so_far = 0;
            // actively slow down
            usleep(job_type_ == CAPTURE_TYPE_PCD_BINARY ? 3000 : 5000);
          }
        }
      }
    }

    if (bks_sum + pcd_saved_packet_headers_[current_packet].item_number == i) {
      bks_sum += pcd_saved_packet_headers_[current_packet].item_number;
      current_packet++;
      inno_log_verify(pcd_packet_so_far_ > current_packet, "%u vs %lu",
                      pcd_packet_so_far_, current_packet);
    } else {
      inno_log_verify(
          bks_sum + pcd_saved_packet_headers_[current_packet].item_number > i,
          "i=%lu bks_sum=%lu current_packet=%lu %u %u", i, bks_sum,
          current_packet, pcd_block_so_far_, pcd_packet_so_far_);
    }
  }
  inno_log_verify(
      pcd_packet_so_far_ == 0 ||
          (pcd_packet_so_far_ == current_packet + 1 &&
           bks_sum + pcd_saved_packet_headers_[current_packet].item_number >=
               pcd_block_so_far_),
      "bks_sum=%lu current_packet=%lu"
      " pcd_packet_so_far_=%u pcd_block_so_far_%u",
      bks_sum, current_packet, pcd_packet_so_far_, pcd_block_so_far_);
  if (byte_so_far != 0) {
    rt = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
        &job_conn_, buffer,
        byte_so_far);
    if (rt < 0) {
      end_worker_();
      inno_log_info("pcd capture %s failed sending %lu bytes.",
                    capture_filename_.c_str(), total_sent);
      return;
    }
  }
  // stop chunks and flush
  rt = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(&job_conn_,
                                                            NULL, 0);
  if (rt < 0) {
    end_worker_();
    inno_log_info("pcd capture %s failed sending %lu bytes.",
                  capture_filename_.c_str(), total_sent);
    return;
  }
  PcServerWsProcessor::flush_buffer_s(&job_conn_);

  end_worker_();
  inno_log_info("pcd capture %s done sending %lu bytes.",
                capture_filename_.c_str(), total_sent);
}

void InnoPcFrameCapture::send_inno_pc_capture_thread_() {
  {
    std::unique_lock<std::mutex> lk(mutex_);
    inno_log_verify(job_type_ == CAPTURE_TYPE_PC ||
                    job_type_ == CAPTURE_TYPE_PC_XYZ,
                    "job_type_ %d", job_type_);
    inno_log_verify(pcd_last_frame_ >= 0, "pcd_last_frame_ %ld",
                    pcd_last_frame_);
  }
  const size_t kMaxChunkLen = 65536 * 4;
  char buffer[kMaxChunkLen];

  // send header
  prepare_http_header(buffer, sizeof(buffer),
                      'F',
                      display_job_duration_,
                      "application/octet-stream",
                      job_type_ == CAPTURE_TYPE_PC ?
                      "inno_pc" :
                      "inno_pc_xyz");
  PcServerWsProcessor::write_buffer_to_ws_socket_full_s(&job_conn_, buffer,
                                                        strlen(buffer));
  size_t total_sent = 0;

  {
    std::unique_lock<std::mutex> lk(mutex_);
    bool need_break = false;
    while (!pc_shutdown_ && !need_break) {
      while (pc_packet_deque_.size() && !pc_shutdown_) {
        InnoDataPacket *q = pc_packet_deque_[0];
        pc_packet_deque_.pop_front();
        if (!q) {
          need_break = true;
          inno_log_verify(pc_packet_deque_.size() == 0, "bad size");
          break;
        }
        lk.unlock();
        int ret;

        if (job_type_ == CAPTURE_TYPE_PC) {
          ret = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
              &job_conn_,
              reinterpret_cast<char*>(q),
              q->common.size);
          total_sent += q->common.size;
        } else {  // job_type_ == CAPTURE_TYPE_PC_XYZ
          InnoDataPacket *qa = reinterpret_cast<InnoDataPacket *>(buffer);
          bool ret2 = InnoDataPacketUtils::convert_to_xyz_pointcloud(
              *q,
              qa,
              sizeof(buffer),
              false);
          inno_log_verify(ret2, "cannot convert to pc_xyz");

          ret = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
              &job_conn_,
              reinterpret_cast<char*>(qa),
              qa->common.size);
          total_sent += qa->common.size;
        }
        pc_packet_pool_->free(q);
        lk.lock();
        if (ret < 0) {
          need_break = true;
          break;
        }
      }
      if (!need_break) {
        pc_cond_.wait(
            lk, [this] { return pc_packet_deque_.size() != 0 ||
                  pc_shutdown_;});
      }
    }
    lk.unlock();
    PcServerWsProcessor::write_chunk_to_ws_socket_full_s(&job_conn_, NULL, 0);
    PcServerWsProcessor::flush_buffer_s(&job_conn_);
    lk.lock();
    clear_pc_packet_deque_w_lock_();
    end_worker_with_lock_();
  }
  inno_log_info("inno_pc capture %s done sending %lu bytes.",
                capture_filename_.c_str(), total_sent);
}

void InnoPcFrameCapture::send_bag_capture_thread_() {
  std::unique_lock<std::mutex> lk(mutex_);
  bool need_break = false;
  while (!pc_shutdown_ && bag_stream_status_ < BAG_STREAM_END &&
         !need_break) {
    while (pc_packet_deque_.size() &&
           bag_stream_status_ < BAG_STREAM_END) {
      InnoDataPacket *q = pc_packet_deque_[0];
      pc_packet_deque_.pop_front();
      if (q == NULL) {
        need_break = true;
        inno_log_verify(pc_packet_deque_.size() == 0, "bad size");
        break;
      }
      lk.unlock();
      do_send_bag_capture_(q);
      pc_packet_pool_->free(q);
      lk.lock();
    }
    if (!need_break) {
      pc_cond_.wait(lk, [this] {
                          return pc_packet_deque_.size() != 0 || pc_shutdown_;
                        });
    }
  }
  lk.unlock();
  if (bag_stream_status_ < BAG_STREAM_END) {
    // end stream
    do_send_bag_capture_(NULL);
  }
  lk.lock();
  clear_pc_packet_deque_w_lock_();
  end_worker_with_lock_();
  delete rosbag_stream_;
  rosbag_stream_ = NULL;
}

int InnoPcFrameCapture::do_send_bag_capture_(InnoDataPacket *pkt) {
  int ret = 0;
  std::unique_lock<std::mutex> lk(mutex_);
  if (bag_stream_status_ == BAG_STREAM_START) {
    lk.unlock();
    char http[1024];
    // send http header
    prepare_http_header(http, sizeof(http),
                        'F',
                        display_job_duration_,
                        "application/octet-stream", "bag");
    ret = PcServerWsProcessor::write_buffer_to_ws_socket_full_s(
        &job_conn_, http,
        strlen(http));
    lk.lock();
    if (ret < 0) {
      inno_log_error("cannot write http header %d", ret);
      bag_stream_status_ = BAG_STREAM_ERROR;
      return -1;
    }
    bag_stream_status_ = BAG_STREAM_SENDING;
  }
  if (bag_stream_status_ < BAG_STREAM_END) {
    lk.unlock();
    if (pkt) {
      ret = rosbag_stream_->add_block(pkt);
      if (ret < 0) {
        inno_log_error("add block return %d", ret);
      }
    } else {
      ret = rosbag_stream_->add_block(NULL);
      bag_stream_status_ = BAG_STREAM_END;
    }
    lk.lock();
    if (ret == RERCORDER_SIZE_LIMIT) {
      bag_stream_status_ = BAG_STREAM_END;
    } else if (ret != 0) {
      bag_stream_status_ = BAG_STREAM_ERROR;
    } else {
      // == 0 do nothing
    }
  }
  if (bag_stream_status_ == BAG_STREAM_END) {
    lk.unlock();
    PcServerWsProcessor::write_chunk_to_ws_socket_full_s(&job_conn_,
                                                               NULL, 0);
    return 1;
  } else if (bag_stream_status_ == BAG_STREAM_ERROR) {
    inno_log_error("Bag stream return error!");
    return -1;
  } else {
    return 0;
  }
}

//
// create thread to send
//
void InnoPcFrameCapture::send_png_capture_thread_() {
  inno_log_info("capture png frame done, sending...");
  inno_log_verify(worker_ == NULL, "worker_");

  this->worker_ = new std::thread([this]() {
    inno_log_info("start work thread.");
    inno_log_verify(this->job_conn_ != NULL, "job_conn_");

    InnoUtils::set_self_thread_priority(-20);

    if (this->job_conn_ != NULL) {
      // xxx todo: use streaming to save memory
      std::vector<char> buf;
      buf.reserve(1024 * 1024 * 6);

      this->png_stream_->save(&buf);

      // send header
      char http[1024];
      prepare_http_header(http, sizeof(http),
                          'F', job_duration_,
                          "application/octet-stream",
                          "png");

      int ret = PcServerWsProcessor::write_buffer_to_ws_socket_full_s(
          &job_conn_, http, strlen(http));

      // send body
      inno_log_info("sending png, size: %lu", buf.size());
      ret = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
          &this->job_conn_, buf.data(), buf.size());

      if (ret < 0) {
        inno_log_warning("Cannot write png recorder to socket.");
        // result = RERCORDER_STREAM_ERROR;
      }

      // end
      PcServerWsProcessor::write_chunk_to_ws_socket_full_s(&job_conn_,
                                                                 NULL, 0);
      PcServerWsProcessor::flush_buffer_s(&job_conn_);
    }

    {
      std::unique_lock<std::mutex> lk(mutex_);
      delete png_stream_;
      png_stream_ = NULL;
      end_worker_with_lock_();
    }
  });

  inno_log_verify(worker_ != NULL, "worker");
}

void InnoPcFrameCapture::end_worker_with_lock_() {
  end_job_with_lock_();
  worker_->detach();
  delete worker_;
  worker_ = NULL;
  pc_cond_.notify_all();
}

void InnoPcFrameCapture::end_worker_() {
  std::unique_lock<std::mutex> lk(mutex_);
  end_worker_with_lock_();
}

void InnoPcFrameCapture::end_job_with_lock_() {
  job_type_ = CAPTURE_TYPE_NONE;
  job_conn_ = NULL;
}

void InnoPcFrameCapture::end_job_() {
  std::unique_lock<std::mutex> lk(mutex_);
  end_job_with_lock_();
}

int InnoPcFrameCapture::write_to_wconn_s_(void *ctx, const void *buf,
                                          size_t buf_len) {
  inno_log_verify(ctx, "ctx");
  return (reinterpret_cast<InnoPcFrameCapture *>(ctx))
      ->write_to_wconn_(buf, buf_len);
}

int InnoPcFrameCapture::write_to_wconn_(const void *buf, size_t buf_len) {
  inno_log_verify(job_conn_, "job_conn");
  int ret = PcServerWsProcessor::write_chunk_to_ws_socket_full_s(
      &job_conn_,
      reinterpret_cast<const char *>(buf),
      buf_len);
  return ret;
}


}  // namespace innovusion
