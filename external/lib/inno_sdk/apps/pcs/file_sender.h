/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef PCS_FILE_SENDER_H_
#define PCS_FILE_SENDER_H_

#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "src/utils/inno_thread.h"
#include "src/ws_utils/server_ws.h"

namespace innovusion {
/**
 * send specified files to conn
 */
class FileSender {
 public:
  FileSender(std::string filenames,
             size_t offset,
             ssize_t length,
             void *conn);
  ~FileSender() {
    if (work_thread_) {
      if (!work_thread_->has_shutdown()) {
        work_thread_->shutdown();
      }
      delete work_thread_;
      work_thread_ = nullptr;
    }
  }
  static void* start_send_s(void *ctx);

 private:
  void* loop_();

 private:
  std::string filename_{};
  size_t offset_{};
  // < 0 means no limit
  ssize_t length_{};
  InnoThread *work_thread_{nullptr};
  std::shared_ptr<WsConnection> conn_;
};

}  // namespace innovusion

#endif  // PCS_FILE_SENDER_H_
