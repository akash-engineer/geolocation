/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_COMMAND_PARSER_H_
#define PCS_COMMAND_PARSER_H_

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>

#include "src/sdk_common/inno_lidar_api.h"

namespace innovusion {


class CommandParser;
class LidarCommandConfig {
  friend CommandParser;

 public:
  LidarCommandConfig();

 public:
  int check() const;
  bool is_local_source() const {
    return data_filename.size() > 0;
  }

 private:
  static const uint16_t kDefaultServerLidarPort_ = 8002;
  static const uint16_t kDefaultClientLidarPort_ = 8010;

 private:
  bool lidar_port_is_default_;

 public:
  std::string lidar_ip;         // n
  uint16_t lidar_port;          // P
  int32_t lidar_udp_port;       // O
  std::string data_filename;    // f
  std::string yaml_filename;    // y
  int file_speed;               // s
  int file_rewind;              // D
  int file_skip;                // S
  int processed;                // c
  int use_xyz;                  // x

  bool set_falcon_eye;                   // e
  double roi_center_h, roi_center_v;
  enum InnoReflectanceMode reflectance;      // F
  enum InnoMultipleReturnMode multireturn;   // M
  uint32_t lidar_id;                         // i
};

class CommandParser {
 public:
  CommandParser(int argc, char *argv[]);
  ~CommandParser();

 private:
  void parse_parameter_(int argc, char *argv[]);
  void usage_();

 public:
  bool ws_enabled() const;
  bool is_client_mode() const;

 private:
  static const uint16_t kDefaultUdpPort_ = 8010;
  static const uint16_t kDefaultServerTcpPort_ = 8010;
  static const uint16_t kDefaultClientTcpPort_ = 8011;

 private:
  int argc_;
  char **argv_;
  bool tcp_port_is_default_;

 public:
  LidarCommandConfig lidar;

  int32_t retry_remote;           // T
  std::string udp_client_ip;      // u
  uint16_t udp_port_data;         // U
  uint16_t udp_port_status;       // W
  uint16_t udp_port_raw;          // k
  uint16_t udp_port_status_local;  // l
  uint16_t udp_port_message;      // w
  uint16_t tcp_port;              // p
  uint32_t status_interval_ms;    // I
  std::string record_inno_pc_filename;   // o
  size_t record_inno_pc_size_in_m;       // m
  int inno_pc_record_npy;                // Y
  std::string record_raw_filename;       // r
  size_t record_raw_size_in_m;           // R
  std::string png_filename;              // t
  std::string rosbag_filename;    // b
  size_t rosbag_size_in_m;        // B
  std::string config_filename;    // g
  std::string config_filename2;   // G
  std::string dtc_filename;       // H

  enum InnoLogLevel debug_level;    // D
  std::string log_filename;         // N
  uint32_t log_file_rotate_number;  // a
  uint32_t log_file_max_size_k;     // A
  std::string error_log_filename;         // C
  uint32_t error_log_file_rotate_number;  // j
  uint32_t error_log_file_max_size_k;     // J
  bool get_version;                 // v
  int show_viewer;                  // V
  int quiet;                        // q
  double sleep_second;              // E

  // set run time of pcs/pcs-client, pcs/pcs-client will exit after run_time_s.
  // unit: second
  std::string run_time_s;           // K
  // If set test_command. pcs/pcs-client will send this command periodically.
  std::string test_command;         // L
  // test command send interval in ms
  std::string test_command_send_interval_ms;  // X

  std::string full_command_line;
};

}  // namespace innovusion

#endif  // PCS_COMMAND_PARSER_H_
