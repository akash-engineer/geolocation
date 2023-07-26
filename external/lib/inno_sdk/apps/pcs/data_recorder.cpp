  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/data_recorder.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <string>

#include "src/sdk_common/inno_lidar_api.h"
#include "src/utils/inno_lidar_log.h"

namespace innovusion {

DataRecorder::DataRecorder(const std::string &f,
                           size_t max_size_in_m,
                           bool processed,
                           bool allow_overwrite)
    : filename_(f)
    , fd_(-1)
    , max_size_(max_size_in_m * 1000 * 1000)
    , written_(0)
    , processed_(processed)
    , allow_overwrite_(allow_overwrite) {
}

DataRecorder::~DataRecorder() {
  close_file_();
}

int DataRecorder::write(const void *buffer, size_t size) {
  ssize_t written;
  if (fd_ < 0) {
    if (filename_.rfind(".") == std::string::npos) {
      filename_ += (processed_ ? ".inno_pc_pkt" : ".inno_raw");
    }
    fd_ = open(filename_.c_str(), O_WRONLY | O_CREAT |
               (allow_overwrite_ ? O_TRUNC : O_EXCL),
               0644);
    if (fd_ < 0) {
      inno_log_error_errno("cannot open data recorder file %s.\n",
                           filename_.c_str());
      return -1;
    }
  }
  if (buffer == NULL || (written_ + size > max_size_ && max_size_ > 0)) {
    close_file_();
    return -1;
  }
  while (-1 == (written = ::write(fd_, buffer, size))
         && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
  }
  if (written < 0) {
    inno_log_error_errno("cannot write data recorder %s, %lu bytes",
                         filename_.c_str(), size);
    close_file_();
    return -1;
  } else {
    written_ += written;
    return 0;
  }
}

void DataRecorder::close_file_() {
  if (fd_ >= 0) {
    close(fd_);
    inno_log_info("data recorder file %s saved. size=%luM",
                  filename_.c_str(), written_/1000/1000);
    fd_ = -1;
  }
}

}  // namespace innovusion
