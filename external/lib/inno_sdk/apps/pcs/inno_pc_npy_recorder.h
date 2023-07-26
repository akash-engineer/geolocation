  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_INNO_PC_NPY_RECORDER_H_
#define PCS_INNO_PC_NPY_RECORDER_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <string>
#include "src/sdk_common/converter/recorder_base.h"
#include "src/sdk_common/inno_lidar_api.h"

namespace innovusion {

class InnoPcNpyRecorder : public RecorderBase {
 private:
  static const size_t kBufferSize = 1024 * 256;

 public:
  explicit InnoPcNpyRecorder(const std::string &filename,
                             ssize_t size_limit_in_m,
                             bool record_npy,
                             bool record_xyz);
  virtual ~InnoPcNpyRecorder();
  virtual int add_block(const InnoDataPacket *pkt);

 protected:
  void add_header_(size_t frame_id_base,
                   size_t timestamp_base_10us);
  void add_cpoint_(void *ctx,
                   const InnoDataPacket &pkt,
                   const InnoBlock &block,
                   const InnoChannelPoint &pt,
                   const InnoBlockFullAngles &,
                   const uint16_t ch,
                   const uint16_t m);
  virtual void close_file_();
  virtual int flush_buffer_();

 protected:
  bool record_npy_;
  bool record_xyz_;
  char pt_buffer_[kBufferSize];
};

}  // namespace innovusion

#endif  // PCS_INNO_PC_NPY_RECORDER_H_
