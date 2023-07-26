/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_LIDAR_SOURCE_H_
#define PCS_LIDAR_SOURCE_H_

#include <mutex>               // NOLINT
#include <string>

#include "pcs/command_parser.h"
#include "src/sdk_common/inno_lidar_api.h"

namespace innovusion {
class LidarSource {
 public:
  explicit LidarSource(const LidarCommandConfig &config,
                       InnoMessageCallback message_callback,
                       InnoDataPacketCallback data_callback,
                       InnoStatusPacketCallback status_callback,
                       void *callback_context);
  ~LidarSource();

 public:
  void start();
  void stop();
  void restart();
  void restart_async();
  void show_lidar_info();
  int set_reflectance_mode(InnoReflectanceMode reflectance);
  int set_return_mode(InnoMultipleReturnMode multireturn);
  int set_roi(double horz_angle, double vert_angle);
  int get_attribute(const std::string &cmd, std::string *result);
  int set_lidar(const std::string &name, const std::string &value);
  int set_recorder_callback(enum InnoRecorderCallbackType type,
                            InnoRecorderCallback callback,
                            void *ctx);
  uint32_t get_id() const {
    return id_;
  }
  int get_mode_status(enum InnoLidarMode *mode,
                      enum InnoLidarMode *pre_mode,
                      enum InnoLidarStatus *ss,
                      uint64_t *in_transition_mode_ms) {
    return inno_lidar_get_mode_status(handle_, mode,
                                      pre_mode, ss,
                                      in_transition_mode_ms);
  }
  int set_config_name_value(const std::string &key,
                            const std::string &value);
  const char *get_sn() const {
    return sn_;
  }

  int get_lidar_handle() const {
    return handle_;
  }

  bool is_live_direct_memory();
  bool is_live_lidar();
  bool is_raw_mem() {
    return protocol_ == INNO_LIDAR_PROTOCOL_RAW_MEM;
  }
  bool is_pc_server();

 private:
  void init_();
  void close_();

 private:
  const LidarCommandConfig &config_;
  InnoMessageCallback message_callback_;
  InnoDataPacketCallback data_callback_;
  InnoStatusPacketCallback status_callback_;
  void *callback_context_;

  int handle_;
  uint32_t id_;
  std::mutex mutex_;
  bool started_;
  enum InnoLidarProtocol protocol_;
  char sn_[InnoStatusPacket::kSnSize];
};

}  // namespace innovusion

#endif  // PCS_LIDAR_SOURCE_H_
