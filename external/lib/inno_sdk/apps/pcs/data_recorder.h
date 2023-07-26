  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_DATA_RECORDER_H_
#define PCS_DATA_RECORDER_H_

#include <stdlib.h>

#include <string>

#include "src/sdk_common/inno_lidar_api.h"

namespace innovusion {
class DataRecorder {
 public:
  explicit DataRecorder(const std::string &f,
                        size_t max_size_in_m,
                        bool processed,
                        bool allow_overwrite);
  ~DataRecorder();

 public:
  int write(const void *buffer, size_t size);

 private:
  void close_file_();

 private:
  std::string filename_;
  int fd_;
  size_t max_size_;
  size_t written_;
  bool processed_;
  bool allow_overwrite_;
};

}  // namespace innovusion

#endif  // PCS_DATA_RECORDER_H_
