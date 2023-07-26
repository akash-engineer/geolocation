/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_LIDAR_CLIENT_COMMUNICATION_H_
#define SDK_CLIENT_LIDAR_CLIENT_COMMUNICATION_H_

#include "utils/net_manager.h"
#include "sdk_common/inno_lidar_api.h"

namespace innovusion {

class LidarClientCommunication: public NetManager {
 public:
  static const int kInvalidVerticalRoi = 1000000;
  static const size_t kSmallBufferSize = 1024;
  static const size_t kMaxUrlSize = 1024;
  static const size_t kMaxReplySize = 4096;

 public:
  LidarClientCommunication(const char *ip,
                           unsigned short port,
                           double timeout_sec);
  ~LidarClientCommunication();

 public:
  int get_attribute(const char *attr, char *buffer_in, size_t buffer_len);
  int get_attribute(const char *attribute, double *value);
  int get_fw_state(char *buffer, size_t buffer_len);
  int get_fw_version(char *buffer, size_t buffer_len);
  int get_sn(char *buffer, size_t buffer_len);
  int get_model(char *buffer, size_t buffer_len);
  int get_debug(InnoLogLevel *debug_level);
  int get_mode_status(enum InnoLidarMode *mode,
                      enum InnoLidarMode *pre_mode,
                      enum InnoLidarStatus *status,
                      uint64_t *in_transition_mode_ms);
  int get_server_udp_ports_ip(int32_t *port_data,
                              int32_t *port_message,
                              int32_t *port_status,
                              char *ip,
                              size_t ip_len,
                              char *ip2,
                              size_t ip2_len);
  int set_server_udp_ports_ip(uint16_t port_data);
  int set_roi(double h_roi, double v_roi);
  int get_roi(double *h_roi, double *v_roi);
  int set_reflectance_mode(InnoReflectanceMode val);
  int set_return_mode(InnoMultipleReturnMode val);
  int set_debug(InnoLogLevel val);
  int set_mode(enum InnoLidarMode val);
  int set_reboot(int value);

  int get_server_udp_raw_port(int32_t *port_raw, char *ip, size_t ip_len);
  int set_server_udp_raw_port(uint16_t port_raw);
  int set_faults_save_raw(uint64_t value);
  int set_attribute_string(const char *attribute,
                           const char *buf);

 private:
  int send_set_command_(const char *set_command);
  int send_get_command_(const char *name,
                        char *buffer,
                        size_t buffer_len);
  int send_get_command_(const char *name,
                        char *buffer,
                        size_t buffer_len,
                        const char *value);
};

}  // namespace innovusion
#endif  // SDK_CLIENT_LIDAR_CLIENT_COMMUNICATION_H_
