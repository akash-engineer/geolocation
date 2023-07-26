/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <fstream>
#include <utility>
#include <vector>

#include "pcs/file_sender.h"
#include "src/ws_utils/server_ws_processor.h"
#include "src/utils/inno_lidar_log.h"
#include "pcs/pc_server_ws_processor.h"

namespace innovusion {

FileSender::FileSender(std::string filenames,
                       size_t offset,
                       ssize_t length,
                       void *conn)
                       : filename_(std::move(filenames)),
                         offset_(offset),
                         length_(length),
                         conn_(*((std::shared_ptr<WsConnection> *)conn)) {
  work_thread_ = new InnoThread("file-sender",
                                0,
                                1,
                                FileSender::start_send_s,
                                this,
                                0,
                                nullptr);
  work_thread_->start();
}

void* FileSender::start_send_s(void *ctx) {
  auto *sender = reinterpret_cast<FileSender *>(ctx);
  sender->loop_();
  return nullptr;
}

void* FileSender::loop_() {
  // open file and send
  std::ifstream ifs;
  ifs.open(filename_.c_str(), std::ifstream ::binary | std::ifstream ::in);
  if (!ifs.is_open()) {
    inno_log_error_errno("open %s error", filename_.c_str());
    ifs.close();
    // return 404
    PcServerWsProcessor::write_buffer_to_ws_socket_full_s(
        &conn_,
        PcServerWsProcessor::bad_reply_400_template,
        strlen(PcServerWsProcessor::bad_reply_400_template));
    PcServerWsProcessor::flush_buffer_s(&conn_);
    return nullptr;
  }

  inno_log_info("start to send file: %s", filename_.c_str());
  char buffer[4096];
  // send header
  int ret = snprintf(buffer, sizeof(buffer),
                     PcServerWsProcessor::reply_capture_template,
                     "application/octet-stream", filename_.c_str());
  inno_log_verify(ret < (ssize_t)sizeof(buffer), "too small %d", ret);
  PcServerWsProcessor::
    write_buffer_to_ws_socket_full_s(&conn_, buffer, strlen(buffer));

  ifs.seekg((int64_t)offset_, std::ifstream::beg);
  ssize_t total_read = 0;
  ssize_t total_sent = 0;
  while (ifs.read(buffer, sizeof(buffer))) {
    total_read += sizeof(buffer);
    if (length_ < 0 || total_read <= length_) {
      PcServerWsProcessor::
      write_chunk_to_ws_socket_full_s(&conn_, buffer, sizeof(buffer));
      total_sent += sizeof(buffer);
    } else {
      // write head buf_sz - (total_read - length_) of buffer
      PcServerWsProcessor::
      write_chunk_to_ws_socket_full_s(
        &conn_, buffer, sizeof(buffer) - (total_read - length_));
      total_sent += sizeof(buffer) - (total_read - length_);
      break;
    }
  }

  if (!ifs) {
    // only read ifs.gcount() byte
    ssize_t read_cnt = ifs.gcount();
    total_read += read_cnt;
    if (length_ < 0 || total_read <= length_) {
      PcServerWsProcessor::
      write_chunk_to_ws_socket_full_s(&conn_, buffer, read_cnt);
      total_sent += read_cnt;
    } else {
      PcServerWsProcessor::
      write_chunk_to_ws_socket_full_s(
        &conn_, buffer, read_cnt - (total_read - length_));
      total_sent += read_cnt - (total_read - length_);
    }
  }
  ifs.close();
  PcServerWsProcessor::write_chunk_to_ws_socket_full_s(&conn_, NULL, 0);
  PcServerWsProcessor::flush_buffer_s(&conn_);
  inno_log_info("send file %s done, total %" PRI_SIZEU " bytes",
                filename_.c_str(), total_sent);
  return nullptr;
}

}  // namespace innovusion
