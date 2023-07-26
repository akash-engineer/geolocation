/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/pcs.h"

#include <sys/resource.h>

#include <memory>
#include <string>
#include <thread>              // NOLINT
#include <vector>

#include "src/sdk_common/converter/cframe_converter.h"
#include "src/sdk_common/converter/png_recorder.h"
#include "src/sdk_common/converter/rosbag_recorder.h"
#include "src/sdk_common/inno_lidar_api.h"
#include "src/utils/inno_lidar_log.h"
#include "src/utils/utils.h"

#include "pcs/command_parser.h"
#include "pcs/data_recorder.h"
#include "pcs/inno_pc_frame_capture.h"
#include "pcs/inno_pc_npy_recorder.h"
#include "pcs/lidar_source.h"
#include "pcs/pc_server_ws_processor.h"
#include "pcs/udp_sender.h"
#include "pcs/version.h"
#include "pcs/command_test.h"
#include "pcs/file_sender.h"

namespace innovusion {

constexpr uint16_t kInvalidUdpPort = 0;

PCS::PCS(const CommandParser &c)
    : cmd_parser_(c)
    , effective_udp_client_ip_()
    , effective_data_port_(0)
    , effective_message_port_(0)
    , effective_status_port_(0)
    , effective_raw_port_(0)
    , data_udp_sender_(NULL)
    , status_udp_sender_(NULL)
    , message_udp_sender_(NULL)
    , status_local_udp_sender_(NULL)
    , raw_udp_sender_(NULL)
    , raw_udp_field_idx_(0)
    , cframe_converter_(NULL)
    , data_recorder_(NULL)
    , bad_data_recorder_(NULL)
    , inno_pc_npy_recorder_(NULL)
    , rosbag_recorder_(NULL)
    , png_recorder_(NULL)
    , frame_capturer_(NULL)
    , ws_(NULL)
    , time_sync_udp_listener_(NULL)
    , dtc_manager_(NULL)
    , cframe_received_(0)
    , shutdown_(false)
    , error_log_offset_(0)
    , error_log_warning_lines_(0)
    , error_log_error_lines_(0)
    , assert_failure_called_(false)
    , starting_log_lines_(0)
    , starting_log_offset_(0)
    , last_time_sync_type_(0) {
  memset(&error_log_, 0, sizeof(error_log_));
  memset(&starting_log_, 0, sizeof(starting_log_));
  // add the callback
  inno_lidar_log_callback(log_callback_s_, this);

  inno_log_info("#############################");
  inno_log_info("###### PCS START ############");
  inno_log_info("#############################");
  inno_log_info("command line: %s", cmd_parser_.full_command_line.c_str());

  struct rlimit limit;
  getrlimit(RLIMIT_STACK, &limit);
  inno_log_info("Stack Limit = %lu and %lu max",
                limit.rlim_cur, limit.rlim_max);

  inno_lidar_setup_sig_handler();

  int e = clock_gettime(CLOCK_MONOTONIC_RAW, &start_timespec_);
  if (e) {
    inno_log_error_errno("clock_gettime return %d", e);
  }

  setup_udps_(cmd_parser_.udp_client_ip,
              cmd_parser_.udp_port_data,
              cmd_parser_.udp_port_message,
              cmd_parser_.udp_port_status);

  if (checke_udp_port_conflict_(cmd_parser_.udp_port_raw)) {
    inno_log_warning("udp_port_raw conflict with other udp_ports: %d",
                     cmd_parser_.udp_port_raw);
  } else {
    setup_raw_udps_(cmd_parser_.udp_client_ip, cmd_parser_.udp_port_raw);
  }

  setup_udp_(std::string("127.0.0.1"),
             cmd_parser_.udp_port_status_local,
             &status_local_udp_mutex_, &status_local_udp_sender_);

  if (cmd_parser_.ws_enabled()) {
    cframe_converter_ = new(CframeConverter);
    inno_log_verify(cframe_converter_, "cframe_converter");
    ws_ = new PcServerWsProcessor(cmd_parser_.tcp_port,
                                  cmd_parser_.lidar.lidar_id,
                                  this);
    inno_log_verify(ws_, "ws");
    frame_capturer_ = new InnoPcFrameCapture(this);
    inno_log_verify(frame_capturer_, "frame_capturer_");
  }

  lidar_ = new LidarSource(cmd_parser_.lidar,
                           message_callback_s_,
                           data_callback_s_, status_callback_s_, this);
  inno_log_verify(lidar_, "lidar");

  dtc_manager_ = new DtcManager(cmd_parser_.dtc_filename.c_str(), lidar_);
  inno_log_verify(dtc_manager_, "dtc_manager");

  if (raw_udp_sender_) {
    lidar_->set_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4,
                                  recorder_callback_s_, this);
  }

  load_config_(cmd_parser_.config_filename.c_str());
  load_config_(cmd_parser_.config_filename2.c_str());

  if (cmd_parser_.record_raw_filename.size()) {
    data_recorder_ = new DataRecorder(cmd_parser_.record_raw_filename,
                                      cmd_parser_.record_raw_size_in_m,
                                      cmd_parser_.lidar.processed,
                                      true);
    inno_log_verify(data_recorder_, "data_recorder");
    lidar_->set_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW,
                                  recorder_callback_s_, this);
  }

  if (!cmd_parser_.lidar.processed) {
    bad_data_recorder_ = new DataRecorder("/tmp/debug.inno_raw", 50,
                                          cmd_parser_.lidar.processed, false);
    inno_log_verify(bad_data_recorder_, "bad_data_recorder");
    lidar_->set_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW3,
                                  recorder_callback_s_, this);
  }

  if (cmd_parser_.record_inno_pc_filename.size()) {
    inno_pc_npy_recorder_ =
        new InnoPcNpyRecorder(cmd_parser_.record_inno_pc_filename,
                              cmd_parser_.record_inno_pc_size_in_m,
                              cmd_parser_.inno_pc_record_npy != 0,
                              cmd_parser_.lidar.use_xyz == 1);
    inno_log_verify(inno_pc_npy_recorder_, "inno_pc_npy_recorder");
  }

  if (cmd_parser_.rosbag_filename.size()) {
    rosbag_recorder_ =
        new RosbagRecorder(cmd_parser_.rosbag_filename.c_str(),
                           NULL, NULL,
                           cmd_parser_.rosbag_size_in_m);
    inno_log_verify(rosbag_recorder_, "rosbag_recorder");
  }

  fw_log_listener_ = new UdpLogListener("fw_log_listener", 7999,
                                        {0, 500 * 1000}, this);
  inno_log_verify(fw_log_listener_, "fw_log_listener");

  if (cmd_parser_.png_filename.empty() == false) {
    png_recorder_ = new PngRecorder(1, 1000 * 100, 1000 * 1000, true);
    inno_log_verify(png_recorder_, "png_filename");
  }

  if (!cmd_parser_.test_command.empty()) {
    inno_log_info("set test command:%s", cmd_parser_.test_command.c_str());
    // set up command test thread
    command_test_ = new CommandTest(this, cmd_parser_.test_command,
                                    cmd_parser_.test_command_send_interval_ms,
                                    cmd_parser_.run_time_s);
    inno_log_verify(command_test_, "command_test");
    it_command_test_ = new InnoThread("command-test", 0,
                                      1, CommandTest::start, command_test_,
                                      0, nullptr);
    inno_log_verify(it_command_test_, "command test thread create failed");
  } else {
    command_test_ = NULL;
    it_command_test_ = NULL;
  }

  memset(&last_status_packet_, 0, sizeof(last_status_packet_));
  memset(&status_packet_, 0, sizeof(status_packet_));
  status_packet_.common.version.magic_number = kInnoMagicNumberStatusPacket;
  status_packet_.common.version.major_version = kInnoMajorVersionStatusPacket;
  status_packet_.common.version.minor_version = kInnoMinorVersionStatusPacket;
  status_packet_.common.size = sizeof(status_packet_);
  status_packet_.common.source_id = cmd_parser_.lidar.lidar_id;

  in_long_duration_test_mode_ = false;
  long_duration_test_log_ = "/mnt/long_duration_test_log";
  long_duration_log_last_save_time_ = 0;
  faults_update_freq_control_ = true;
}

PCS::~PCS() {
  inno_log_info("PCS destructor");

  // have to stop log call back before move other services
  inno_lidar_log_callback(NULL, NULL);

  if (dtc_manager_) {
    delete dtc_manager_;
    dtc_manager_ = NULL;
  }
  if (ws_) {
    delete ws_;
    ws_ = NULL;
  }
  if (lidar_) {
    delete lidar_;
  }
  {
    std::unique_lock<std::mutex> lk(data_udp_mutex_);
    if (data_udp_sender_) {
      delete data_udp_sender_;
      data_udp_sender_ = NULL;
    }
  }
  {
    std::unique_lock<std::mutex> lk(status_udp_mutex_);
    if (status_udp_sender_) {
      delete status_udp_sender_;
      status_udp_sender_ = NULL;
    }
  }
  {
    std::unique_lock<std::mutex> lk(status_local_udp_mutex_);
    if (status_local_udp_sender_) {
      delete status_local_udp_sender_;
      status_local_udp_sender_ = NULL;
    }
  }
  {
    std::unique_lock<std::mutex> lk(raw_udp_mutex_);
    if (raw_udp_sender_) {
      delete raw_udp_sender_;
      raw_udp_sender_ = NULL;
      raw_udp_field_idx_ = 0;
    }
  }
  if (cframe_converter_) {
    delete cframe_converter_;
    cframe_converter_ = NULL;
  }
  if (data_recorder_) {
    delete data_recorder_;
    data_recorder_ = NULL;
  }
  if (bad_data_recorder_) {
    delete bad_data_recorder_;
    bad_data_recorder_ = NULL;
  }
  if (inno_pc_npy_recorder_) {
    delete inno_pc_npy_recorder_;
    inno_pc_npy_recorder_ = NULL;
  }
  if (fw_log_listener_) {
    delete fw_log_listener_;
    fw_log_listener_ = NULL;
  }
  if (it_command_test_) {
    it_command_test_->shutdown();
    delete it_command_test_;
    it_command_test_ = nullptr;
  }
  if (command_test_) {
    delete command_test_;
    command_test_ = nullptr;
  }
  if (time_sync_udp_listener_) {
    time_sync_udp_listener_->stop();
    delete time_sync_udp_listener_;
    time_sync_udp_listener_ = nullptr;
  }
  {
    std::unique_lock<std::mutex> lk(message_udp_mutex_);
    if (message_udp_sender_) {
      delete message_udp_sender_;
      message_udp_sender_ = NULL;
    }
  }
}

int PCS::load_config_line_(const char *l) {
  std::string line(l);
  InnoUtils::trim_space(&line);
  const char *lidar_section_start = "Lidar_";

  size_t eq_start = line.find("=");

  if (eq_start == std::string::npos) {
    inno_log_warning("suspicious line %s", l);
    return -1;
  }

  std::string key = line.substr(0, eq_start);
  std::string value = line.substr(eq_start + 1);
  InnoUtils::trim_space(&key);
  InnoUtils::trim_space(&value);

  if (key.find(lidar_section_start, 0) != 0) {
    inno_log_trace("not start with %s line %s", lidar_section_start, l);
    return -2;
  } else {
    inno_log_info("lidar config file line: %s", line.c_str());
    return lidar_->set_config_name_value(key, value);
  }
}

void PCS::load_config_(const char *filename) {
  if (!filename || strlen(filename) == 0) {
    return;
  }
  FILE* file = fopen(filename, "r");
  int result = 0;
  if (file) {
    char line[1024];
    while (fgets(line, sizeof(line), file)) {
      int ret = load_config_line_(line);
      if (ret != 0 && ret != -2) {
        result = ret;
      }
    }
    fclose(file);
  } else {
    result = -1;
    inno_log_warning("cannot open %s", filename);
  }
  bool cond = result != 0;
  std::string str(filename);
  if (str.find("inno_pc_server.config") != std::string::npos) {  // PCS config
    dtc_manager_->set_raw_fault(INNO_LIDAR_IN_FAULT_CONFIG1, cond);
  } else if (str.find("config_firmware") != std::string::npos) {  // FW cfg
    dtc_manager_->set_raw_fault(INNO_LIDAR_IN_FAULT_CONFIG2, cond);
  }
  return;
}

void PCS::setup_udp_(const std::string &udp_ip, uint16_t port,
                     std::mutex *mutex, UdpSender **sender) {
  inno_log_verify(sender, "sender");
  inno_log_verify(mutex, "mutex");
  inno_log_info("request to create udp_sender to %s:%hu", udp_ip.c_str(), port);
  {
    // note that we can not do any log while holding this mutex
    std::unique_lock<std::mutex> lk(*mutex);
    if (*sender != NULL) {
      delete *sender;
      *sender = NULL;
    }

    // when udp_ip is "0.0.0.0", not create udp socket
    if (port == kInvalidUdpPort || !InnoUtils::check_ip_valid(udp_ip.c_str())) {
      return;
    }

    *sender = new UdpSender(udp_ip, port);
  }

  inno_log_verify(*sender, "udp_sender_ %s %hu", udp_ip.c_str(), port);
  inno_log_info("create udp_sender to %s:%hu", udp_ip.c_str(), port);
}

//
// raw_port can not use lidar data/message/status port.
//
bool PCS::checke_udp_port_conflict_(uint16_t raw_port) {
  // udp broadcast
  // if (effective_udp_raw_ip_ != effective_udp_client_ip_) {
  //   return false;
  // }

  if (raw_port == kInvalidUdpPort) {
    return false;
  }

  return (raw_port == effective_data_port_ ||
          raw_port == effective_message_port_ ||
          raw_port == effective_status_port_);
}

void PCS::setup_udps_(const std::string &udp_ip, uint16_t data_port,
                      uint16_t message_port, uint16_t status_port) {
  effective_udp_client_ip_ = udp_ip;
  effective_data_port_ = data_port;
  effective_message_port_ = message_port;
  effective_status_port_ = status_port;

  // if conflict, disable raw port
  if (checke_udp_port_conflict_(effective_raw_port_)) {
    setup_raw_udps_(udp_ip, kInvalidUdpPort);
  }

  setup_udp_(udp_ip, message_port, &message_udp_mutex_, &message_udp_sender_);
  setup_udp_(udp_ip, data_port, &data_udp_mutex_, &data_udp_sender_);
  setup_udp_(udp_ip, status_port, &status_udp_mutex_, &status_udp_sender_);
}

void PCS::setup_raw_udps_(const std::string &udp_ip, uint16_t raw_port) {
  effective_udp_raw_ip_ = udp_ip;
  effective_raw_port_ = raw_port;

  setup_udp_(udp_ip, raw_port, &raw_udp_mutex_, &raw_udp_sender_);

  // recorder_callback_ will disable raw4 callback
  // so we must set it again.
  if (lidar_ && raw_udp_sender_) {
    lidar_->set_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4,
                                  recorder_callback_s_, this);
  }
}

int PCS::setup_time_sync_listener_(const std::string &name,
                                   const std::string &ip_in,
                                   uint16_t port_in,
                                   const std::string &ip_out,
                                   uint16_t port_out) {
  if (port_in <= 0 || port_out <= 0) {
    inno_log_error("invalid params to setup time sync listener, "
                   "ip_in:%s, port_in:%d, ip_out:%s, port_out:%d",
                   ip_in.c_str(), port_in, ip_out.c_str(), port_out);
    return -1;
  }

  {
    std::unique_lock<std::mutex> lk(time_sync_udp_listener_mutex_);
    if (time_sync_udp_listener_) {
      inno_log_info("remove current time sync listener");
      time_sync_udp_listener_->stop();
      delete time_sync_udp_listener_;
      time_sync_udp_listener_ = nullptr;
    }
    time_sync_udp_listener_ = new TimeSyncUdpListener(name,
                                                      ip_in,
                                                      port_in,
                                                      {0, 500 * 1000},
                                                      ip_out,
                                                      port_out,
                                                      this);
    time_sync_udp_listener_->start();
  }
  return 0;
}

int PCS::get_pcs(const std::string &name,
                 const std::string &value,
                 std::string *result, void *conn,
                 bool external) {
  bool print = external;
  if (result->size() > 0) {
    inno_log_warning("one get_ commad at a time. %s %s",
                     name.c_str(), result->c_str());
    return -1;
  }
  int ret = 0;
  if (name == "usage") {
    *result +=
        "http://<LIDAR-IP>:8010/capture/?type=<TYPE>&duration=<DURATION>\n"
        " <TYPE> is one of the following: pcd bag inno_pc inno_raw\n"
        " <DURATION> is in number of frames (for pcd and inno_pc) or MBytes\n"
        " example: curl "
        "\"http://172.168.1.10:8010/capture/?type=pcd&duration=15\" -O -J\n"
        "\n"
        "http://<LIDAR-IP>:8010/command/?<COMMAND>\n"
        " <COMMAND> is one of the following:\n "
        "get_usage "
        "get_commands get_enabled set_stop\n "
        "get_sw_version "
        "get_fw_version "
        "get_sdk_api_version "
        "get_sdk_version "
        "get_sdk_build_tag "
        "get_sdk_build_time\n "
        "get_sn "
        "get_yaml "
        "get_model "
        "get_temperature "
        "get_detector_temps "
        "get_motor_speeds "
        "get_mode_status set_mode\n "
        "set_reboot "
        "get_roi set_roi "
        "get_frame_rate "
        "get_reflectance_mode set_reflectance_mode "
        "get_return_mode set_return_mode\n "
        "get_command_line "
        "get_debug "
        "get_status_interval_ms "
        "get_udp_ports_ip "
        "get_udp_raw_port "
        "get_udp_raw_data "
        "get_udp_ip\n "
        "get_uptime get_time "
        "get_pid "
        "get_system_stats "
        "get_output_stats\n "
        "get_cpu_read "
        "get_cpu_signal "
        "get_cpu_angle "
        "get_cpu_n0 "
        "get_cpu_n1 "
        "get_cpu_deliver\n "
        "get_stage_read "
        "get_stage_signal "
        "get_stage_angle "
        "get_stage_n0 "
        "get_stage_n1 "
        "get_stage_deliver\n "
        "get_support "
        "get_error_log "
        "get_error_log_size "
        "get_error_log_file\n "
        "get_starting_log\n"
        "get_time_sync_type\n "
        "get_inner_faults "
        "get_inner_faults_all_info "
        "set_inner_fault "
        "set_clear_inner_fault "
        "set_clear_inner_faults\n "
        "set_long_duration_test "
        "set_clear_long_duration_test\n"
        "set_faults_save_raw\n"
        "\n";
  } else if (name == "commands" || name == "command") {
    *result += "get_usage "
               "get_commands "
               "get_enabled "
               "set_stop "
               "get_sw_version "
               "get_fw_version "
               "get_sdk_api_version "
               "get_sdk_version "
               "get_sdk_build_tag "
               "get_sdk_build_time "
               "get_sn "
               "get_yaml "
               "get_model "
               "set_mode "
               "set_reboot "
               "get_temperature "
               "get_detector_temps "
               "get_motor_speeds "
               "get_mode_status "
               "get_roi "
               "set_roi "
               "get_frame_rate "
               "get_reflectance_mode "
               "set_reflectance_mode "
               "get_return_mode "
               "set_return_mode "
               "get_command_line "
               "get_debug "
               "get_status_interval_ms "
               "get_udp_ports_ip "
               "get_udp_raw_port "
               "get_udp_raw_data "
               "get_udp_ip "
               "get_time "
               "get_uptime "
               "get_pid "
               "get_system_stats "
               "get_output_stats "
               "get_cpu_read "
               "get_cpu_signal "
               "get_cpu_angle "
               "get_cpu_n0 "
               "get_cpu_n1 "
               "get_cpu_deliver "
               "get_stage_read "
               "get_stage_signal "
               "get_stage_angle "
               "get_stage_n0 "
               "get_stage_n1 "
               "get_stage_deliver "
               "get_support "
               "get_error_log "
               "get_error_log_size "
               "get_error_log_file "
               "get_starting_log "
               "get_time_sync_type "
               "get_inner_faults "
               "get_inner_faults_all_info "
               "set_inner_fault "
               "set_clear_inner_fault "
               "set_clear_inner_faults "
               "set_long_duration_test "
               "set_clear_long_duration_test "
               "set_faults_save_raw ";
  } else if (name == "sw_version") {
    *result += std::string("VERSION: ") + innovusion_version +
               "\nBUILD_TAG: " + innovusion_build_tag +
               "\nBUILD_TIME: " + innovusion_build_time +
               "\nAPI: " + inno_api_version() +
               "\nAPI_BUILD_TAG: " + inno_api_build_tag() +
               "\nAPI_BUILD_TIME: " + inno_api_build_time() +
               "\n";
  } else if (name == "pid") {
    int pid = getpid();
    char buf[16];
    uint32_t r = snprintf(buf, sizeof(buf), "%d", pid);
    inno_log_verify(r < sizeof(buf), "impossible");
    *result += buf;
  } else if (name == "time") {
    struct tm newtime;
    time_t ltime;
    char buf[128];
    ltime = time(&ltime);
    localtime_r(&ltime, &newtime);
    *result = std::string("local_time=") +
              asctime_r(&newtime, buf);
    struct timespec current_timespec;
    int e = clock_gettime(CLOCK_REALTIME, &current_timespec);
    if (!e) {
      uint32_t r = snprintf(buf, sizeof(buf), "%f",
                            current_timespec.tv_sec +
                            current_timespec.tv_nsec / 1000000000.0);
      inno_log_verify(r < sizeof(buf), "impossible");
      *result += " EPOCH=";
      *result += buf;
    }
  } else if (name == "uptime") {
    struct timespec stop_timespec;
    int e = clock_gettime(CLOCK_MONOTONIC_RAW, &stop_timespec);
    if (e) {
      inno_log_error_errno("clock_gettime return %d", e);
      *result += "0";
    } else {
      char buf[32];
      double t = stop_timespec.tv_sec +
                 stop_timespec.tv_nsec / 1000000000.0 -
                 start_timespec_.tv_sec -
                 start_timespec_.tv_nsec / 1000000000.0;
      uint32_t r = snprintf(buf, sizeof(buf), "%f", t);
      inno_log_verify(r < sizeof(buf), "impossible");
      *result += buf;
    }
  } else if (name == "command_line") {
    *result += cmd_parser_.full_command_line;
  } else if (name == "sdk_api_version") {
    *result += inno_api_version();
  } else if (name == "sdk_build_tag" || name == "sdk_version") {
    *result += inno_api_build_tag();
  } else if (name == "sdk_build_time") {
    *result += inno_api_build_time();
  } else if (name == "lidar_id") {
    *result += std::to_string(cmd_parser_.lidar.lidar_id);
  } else if (name == "debug") {
    *result += std::to_string(inno_log_level_g);
  } else if (name == "status_interval_ms") {
    *result += std::to_string(cmd_parser_.status_interval_ms);
  } else if (name == "udp_ip") {
    if (data_udp_sender_) {
      *result += data_udp_sender_->get_udp_ip_string();
    } else if (status_udp_sender_) {
      *result += status_udp_sender_->get_udp_ip_string();
    } else if (message_udp_sender_) {
      *result += message_udp_sender_->get_udp_ip_string();
    } else {
      /*when all data_*_sender_ are NULL,
      then return the initial udp_client_ip value or inputed value*/
      *result += cmd_parser_.udp_client_ip;
    }
  } else if (name == "udp_ports_ip") {
    /*if start pcs without --udp-ip parameter, shouldn't return 0,0,0,0
      just replace udp-ip with initial value 0.0.0.0*/
    std::string source_ip =
        conn ? PcServerWsProcessor::get_source_ip_string_s(conn).c_str() : "";
    *result += std::to_string(effective_data_port_) + "," +
                std::to_string(effective_status_port_) + "," +
                std::to_string(effective_message_port_) + "," +
                effective_udp_client_ip_;

    if (data_udp_sender_) {
      *result += "," + data_udp_sender_->get_udp_ip_string();
      if (data_udp_sender_->is_multicast()) {
        *result += "," + source_ip;
      }
    } else {
      *result += "," + cmd_parser_.udp_client_ip + "," + source_ip;
    }
  } else if (name == "udp_raw_port") {
    *result +=
        std::to_string(effective_raw_port_) + "," + effective_udp_raw_ip_;
  } else if (name == "udp_raw_data") {
    if (this->raw_udp_sender_) {
      *result += "Sending raw data to udp " + effective_udp_raw_ip_ + ":" +
                 std::to_string(effective_raw_port_);
      ret = lidar_->set_lidar("command", "get_udp_raw_data");
    } else {
      *result +=
          "The raw udp port - " + std::to_string(effective_raw_port_) +
          " is invalid, please set unique port with set_udp_raw_port command";
    }
  } else if (name == "support") {
    std::vector<std::string> cmds =
        {"time",
         "sdk_api_version",
         "sdk_build_tag",
         "fw_version",
         "command_line",
         "sn",
         "model",
         "mode_status",
         "reboot"
         "roi",
         "frame_rate",
         "reflectance_mode",
         "return_mode",
         "udp_ports_ip",
         "udp_raw_port",
         "debug",
         "uptime",
         "system_stats",
         "output_stats",
         "cpu_read",
         "cpu_signal",
         "cpu_angle",
         "cpu_n0",
         "cpu_n1",
         "cpu_deliver",
         "stage_read",
         "stage_signal",
         "stage_angle",
         "stage_n0",
         "stage_n1",
         "stage_deliver",
         "inner_faults",
        };
    for (std::vector<std::string>::iterator t = cmds.begin();
         t != cmds.end(); ++t) {
      std::string str;
      get_pcs(*t, std::string(""), &str, conn, false);
      *result += *t + ": " + str + "\n";
      if (str.empty() || str.back() != '\n') {
        *result += "\n";
      }
    }
  } else if (name == "error_log") {
    std::unique_lock<std::mutex> lk(mutex_);
    size_t sz = strtoul(value.c_str(), NULL, 0);
    if (sz <= error_log_offset_) {
      *result = error_log_ + sz;
    } else {
      *result = "";
    }
    print = false;
  } else if (name == "error_log_size") {
    std::unique_lock<std::mutex> lk(mutex_);
    *result = std::to_string(error_log_offset_) + " " +
              std::to_string(error_log_warning_lines_) + " " +
              std::to_string(error_log_error_lines_);
  } else if (name == "error_log_file") {
    *result = "";
    if (cmd_parser_.error_log_file_rotate_number > 0) {
      for (int k = cmd_parser_.error_log_file_rotate_number - 1;
           k >= 0; k--) {
        std::string f = cmd_parser_.error_log_filename;
        if (k > 0) {
          f += ".";
          f += std::to_string(k);
        }
        std::ifstream ifs(f);
        std::string content((std::istreambuf_iterator<char>(ifs)),
                             (std::istreambuf_iterator<char>()));
        *result += content;
      }
    }
  } else if (name == "starting_log") {
    std::string log;
    static const char *starting_log_boundary_ = "\n### STARTING LOG ###\n";
    {
      std::unique_lock<std::mutex> lk(mutex_);
      size_t user_specified_offset = strtoul(value.c_str(), nullptr, 0);
      if (user_specified_offset <= starting_log_offset_) {
        log = std::string(starting_log_boundary_) +
            (starting_log_ + user_specified_offset) +
            starting_log_boundary_;
        // xxx todo return more information about starting log in result
        uint64_t remain_size = starting_log_offset_ - user_specified_offset;
        *result = std::string(starting_log_ + user_specified_offset,
                              remain_size <= 1000 ? remain_size : 1000);
      } else {
        log = "";
        *result = "";
      }
    }
    write_log_(InnoLogLevel::INNO_LOG_LEVEL_INFO, 0,
               "", "", log.c_str(), true);
    print = false;
  } else if (name == "time_sync_type") {
    *result = std::to_string(last_status_packet_.common.timestamp_sync_type);
  } else if (name == "inner_faults") {  // return faults(current or history)
    inno_log_verify(dtc_manager_, "dtc_manager_");
    *result = dtc_manager_->get_inner_faults_info();
  } else if (name == "inner_faults_all_info") {  // return all info
    inno_log_verify(dtc_manager_, "dtc_manager_");
    *result = dtc_manager_->get_inner_faults_all_info();
  } else if (name == "log_snapshot_status") {
    if (lidar_->is_live_direct_memory()) {
      {
        std::unique_lock<std::mutex> lk(log_snapshot_mutex_);
        *result = std::to_string(log_snapshot_status_);
      }
      *result += "\n0-success,1-none snapshot did,2-doing,3-failed";
    } else {
      lidar_->set_lidar(name, value);
    }
  } else {
    ret = lidar_->get_attribute(name, result);
  }
  if (print) {
    inno_log_info("command get_%s ret=%d result= %s",
                  name.c_str(), ret, result->c_str());
  }
  return ret;
}

int PCS::set_pcs(const std::string &name,
                  const std::string &value, void *conn) {
  std::string source_ip;
  if (conn) {
    source_ip =
        PcServerWsProcessor::get_source_ip_string_s(conn).c_str();
    inno_log_info("get set_%s request %s from %s", name.c_str(), value.c_str(),
                  source_ip.c_str());
  }

  int ret = 0;
  if (name == "debug") {
    int debug_level;
    if (!InnoUtils::is_unsinged_decimal_integer(value) ||
        sscanf(value.c_str(), "%d", &debug_level) != 1) {
      inno_log_warning("Invalid query %s %s", name.c_str(), value.c_str());
    } else {
      inno_lidar_set_log_level((InnoLogLevel)debug_level);
    }
  } else if (name == "stop") {
    inno_log_info("received stop command");
    exit(0);
  } else if (name == "restart2") {
    inno_log_info("received restart2 command");
    stop();
  } else if (name == "pause") {
    lidar_->stop();
  } else if (name == "resume") {
    lidar_->start();
  } else if (name == "restart") {
    inno_log_info("received restart command, XXX todo, still have problem");
    enum InnoLidarMode current_mode;
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    uint64_t transition_ms;
    int ret_val = lidar_->get_mode_status(&current_mode, &pre_mode,
                                      &status, &transition_ms);
    if (ret_val != 0 || status != INNO_LIDAR_STATUS_NORMAL ||
       (current_mode != INNO_LIDAR_MODE_WORK_NORMAL &&
        current_mode != INNO_LIDAR_MODE_WORK_CALIBRATION &&
        current_mode != INNO_LIDAR_MODE_WORK_QUIET &&
        current_mode != INNO_LIDAR_MODE_WORK_INTERNAL_1)) {
      inno_log_warning("ignore current restart command, "
                       "ret: %d, current mode: %d, pre_mode: %d, "
                       "status: %d, transition time: %lums, "
                       "please try later", ret_val, current_mode,
                        pre_mode, status, transition_ms);
      ret = -1;
    } else {
      lidar_->stop();
      lidar_->start();
    }
  } else if (name == "udp_ports_ip") {
    int32_t port_data;
    int32_t port_message;
    int32_t port_status;

    char ip[20]{0};

    if (!InnoUtils::check_ip_valid(cmd_parser_.udp_client_ip.c_str())) {
      int got = sscanf(value.c_str(), "%d,%d,%d,%18s", &port_data,
                       &port_message, &port_status, ip);
      if (got == 4 && strlen(ip) > 0) {
        // setup_udps_(std::string(ip), port_data, port_message, port_status);
        inno_log_info("DO NOT use ip %s in request", ip);
      } else if (got == 3) {
        setup_udps_(source_ip, port_data, port_message, port_status);
        inno_log_info("use ip %s from connection", source_ip.c_str());
      } else {
        inno_log_warning("ignore, invalid request");
        ret = -1;
      }
    } else {
      inno_log_info("udp_client_ip is fixed: %s. ignore request",
                    cmd_parser_.udp_client_ip.c_str());
    }
  } else if (name == "udp_raw_port") {
    int32_t port_raw{kInvalidUdpPort};

    int got = sscanf(value.c_str(), "%d", &port_raw);
    if (got == 1) {
      if (checke_udp_port_conflict_(port_raw)) {
        inno_log_warning("udp_port_raw conflict with other udp_ports: %d",
                         port_raw);
      } else {
        setup_raw_udps_(source_ip, port_raw);
      }
    } else {
      inno_log_warning("ignore, invalid request");
      ret = -1;
    }
  } else if (name == "inner_fault") {
    uint32_t fault = atoi(value.c_str());
    if (fault < 0 || fault > INNO_LIDAR_IN_FAULT_MAX) {
      ret = -1;
    } else {
      inno_log_verify(dtc_manager_, "dtc_manager_");
      // test interface, only for debug, should be deleted before release
      InnoLidarInFault i = InnoLidarInFault(fault);
      dtc_manager_->set_fault_external(i);
    }
  } else if (name == "inner_faults") {
    inno_log_verify(dtc_manager_, "dtc_manager_");
    dtc_manager_->set_fault_external();
  } else if (name == "clear_inner_fault") {
    uint32_t fault = atoi(value.c_str());
    if (fault < 0 || fault > INNO_LIDAR_IN_FAULT_MAX) {
      ret = -1;
    } else {
      inno_log_verify(dtc_manager_, "dtc_manager_");
      InnoLidarInFault i = InnoLidarInFault(fault);
      dtc_manager_->heal_faults_external(i);
    }
  } else if (name == "clear_inner_faults") {
    inno_log_verify(dtc_manager_, "dtc_manager_");
    dtc_manager_->heal_faults_external();
  } else if (name == "long_duration_test") {
    // in long duration test mode, pcs write heart beat info into static memory
    // for checking if it is working normal while lidar become unconnected
    // for unkonwn reason.
    set_up_long_duration_test_mode_();
  } else if (name == "clear_long_duration_test") {
    clear_long_duration_test_mode_();
  } else if (name == "stop_raw_raw_capture" || name == "stop_capture") {
    if (value == "1") {
      // stop current raw_raw capturing task
      if (lidar_->is_live_direct_memory() && frame_capturer_) {
        frame_capturer_->stop_capture_force();
        ret = 0;
      } else {
        ret = lidar_->set_lidar(name, value);
      }
    } else {
      inno_log_error("invalid parameter %s, only support 1", value.c_str());
      ret = -1;
    }
  } else if (name == "time_sync_check") {
    if (!lidar_->is_live_direct_memory()) {
      return inno_lidar_set_attribute_string(lidar_->get_lidar_handle(),
                                             "time_sync_check", value.c_str());
    }
    std::vector<std::string> values = InnoUtils::split(value, ",");
    size_t got = values.size();
    if (values.size() == 4
        && values[1].length() > 0
        && values[2].length() > 0) {
      return setup_time_sync_listener_("time_sync_listener",
                                values[0],
                                std::stoul(values[1]),
                                values[2],
                                std::stoul(values[3]));
    } else {
      inno_log_warning("ignore, invalid request, got %" PRI_SIZEU " params",
                       got);
      ret = -1;
    }
  } else if (name == "log_snapshot") {
    // snapshot pcs/fw/uds logs, compress
    // parameter value is the path to store compressed snapshot,
    // default is /mnt/log_snapshot_%YYYY_%MM_%DD_%HH_%mm_%SS.tgz
    if (lidar_->is_live_direct_memory()) {
      {
        std::unique_lock<std::mutex> lk(log_snapshot_mutex_);
        if (log_snapshot_status_ == LOG_SNAPSHOT_DOING) {
          inno_log_warning("is already started doing a snapshot");
          return 503;
        }
        log_snapshot_status_ = LOG_SNAPSHOT_DOING;
      }
      ret = log_snapshot_(value);
      if (ret) {
        std::unique_lock<std::mutex> lk(log_snapshot_mutex_);
        log_snapshot_status_ = LOG_SNAPSHOT_FAILED;
      }
    } else {
      ret = lidar_->set_lidar(name, value);
    }
  } else {
    ret = lidar_->set_lidar(name, value);
  }

  if (ret) {
    inno_log_warning("command set_%s to %s ret=%d",
                     name.c_str(), value.c_str(), ret);
  }
  return ret;
}

int PCS::log_snapshot_(const std::string &value) {
  int ret = 0;
  std::string path;
  std::string tmp_file = "/tmp/log_snapshot.tgz";
  std::string file_name = "/log_snapshot_"
      + InnoUtils::get_current_time_str("%Y_%m_%d_%H_%M_%S")
      + ".tgz";
  if (value.empty() || value == "1") {
    // default path
    path = "/mnt";
  } else {
    // user specified path, only /tmp and path under /tmp are valid
    path = value;
    if (path != "/tmp" &&
        !InnoUtils::start_with(path.c_str(), "/tmp/")) {
      inno_log_error("path %s is invalid, must under /tmp",
                     path.c_str());
      return 400;
    }
    if (access(path.c_str(), F_OK) != 0) {
#ifndef __MINGW64__
      if (mkdir(path.c_str(), S_IRWXU) != 0) {
        inno_log_error("create %s failed", path.c_str());
        return 400;
      }
#else
      if (::mkdir(path.c_str()) != 0) {
        inno_log_error("create %s failed", path.c_str());
        return 400;
      }
#endif
    }
  }
  path += file_name;
  std::thread t1([this, tmp_file, path]() {
      log_snapshot_do_(tmp_file, path);
    });
  t1.detach();
  return ret;
}

void PCS::log_snapshot_do_(std::string tmp_file, std::string path) {
  inno_log_verify(!tmp_file.empty(), "tmp_file is empty");
  inno_log_verify(!path.empty(), "path is empty");
  int ret;
  ret = system(("/app/pointcloud/log_snapshot.sh "
                + tmp_file + " " + path).c_str());
  ret = WEXITSTATUS(ret);
  if (ret == 0) {
    inno_log_info("do log snapshot %s down", path.c_str());
    std::unique_lock<std::mutex> lk(log_snapshot_mutex_);
    log_snapshot_status_ = LOG_SNAPSHOT_SUCCESS;
  } else if (ret == 1) {
    inno_log_error("do log snapshot %s failed: compress logs failed.",
                   path.c_str());
    std::unique_lock<std::mutex> lk(log_snapshot_mutex_);
    log_snapshot_status_ = LOG_SNAPSHOT_FAILED;
  } else if (ret == 2) {
    // There is no enough space in /mnt
    inno_log_warning("do log snapshot %s only for latest part:"
                     " /mnt/ space is tight.",
                     path.c_str());
    ret = 0;
    std::unique_lock<std::mutex> lk(log_snapshot_mutex_);
    log_snapshot_status_ = LOG_SNAPSHOT_SUCCESS;
  } else {
    inno_log_error("do log snapshot %s failed: ret = %d",
                   path.c_str(), ret);
  }
}

int PCS::add_capture_job(const std::string &type, const std::string &duration,
                         void *conn) {
  if (frame_capturer_) {
    return frame_capturer_->add_capture_job(type, duration, conn);
  } else {
    return -1;
  }
}

const char *PCS::get_sn() const {
  return lidar_->get_sn();
}

int PCS::set_recorder_callback(enum InnoRecorderCallbackType type,
                               InnoRecorderCallback callback,
                               void *ctx) {
  inno_log_verify(lidar_, "lidar_");
  return lidar_->set_recorder_callback(type, callback, ctx);
}

InnoThread *PCS::get_command_test_thread() const {
  return it_command_test_;
}

PcServerWsProcessor *PCS::get_ws() const {
  return ws_;
}

int PCS::set_lidar_attribute(const std::string &name,
                              const std::string &value) {
  return lidar_->set_lidar(name, value);
}

void PCS::pause_lidar() {
  lidar_->stop();
}

void PCS::resume_lidar() {
  lidar_->start();
}

void PCS::write_log_(int level, int code,
                     const char *header1,
                     const char *header2,
                     const char *message,
                     bool is_log) {
  static const uint32_t kMaxSize = 65000;
  union {
    InnoDataPacket packet;
    char buf[kMaxSize];
  };
  InnoMessage &msg = packet.messages[0];
  memset(buf, 0, sizeof(packet));
  packet.common.version.magic_number = kInnoMagicNumberDataPacket;
  packet.common.version.major_version = kInnoMajorVersionDataPacket;
  packet.common.version.minor_version = kInnoMinorVersionDataPacket;
  packet.common.source_id = cmd_parser_.lidar.lidar_id;
  // xxx todo:
  // packet.common.timestamp_sync_type = ;
  // packet.common.ts_start_us = ;
  // packet.common.lidar_mode = ;
  // packet.common.lidar_status = ;

  {
    std::unique_lock<std::mutex> lk(mutex_);
    packet.idx = is_log ? message_id_++ : message_log_id_++;
    if (level <= INNO_LOG_LEVEL_DEBUG && level >= 0) {
      if (level <= INNO_LOG_LEVEL_ERROR) {
        error_log_error_lines_++;
      } else {
        error_log_warning_lines_++;
      }
      if (error_log_offset_ < sizeof(error_log_) - 1) {
        uint32_t s = sizeof(error_log_) - 1 - error_log_offset_;
        int k = snprintf(error_log_ + error_log_offset_,
                         s,
                         "%s %s %s %s\n",
                         header1, inno_log_header_g[level],
                         header2, message);
        if (k < (int32_t)s) {
          error_log_offset_ += k;
        } else {
          error_log_offset_ += s - 1;
        }
      }
    }
  }

  packet.type = is_log ? INNO_ITEM_TYPE_MESSAGE_LOG : INNO_ITEM_TYPE_MESSAGE;
  packet.item_number = 1;
  msg.src = cmd_parser_.lidar.lidar_id;
  msg.id = packet.idx;
  msg.level = level;
  msg.code = code;
  int max_size = kMaxSize - sizeof(InnoDataPacket) - sizeof(InnoMessage);
  int sz = 0;
  if (code == INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT) {
    sz = snprintf(&msg.content[0], max_size,
                    "%s", message);
  } else {
    sz = snprintf(&msg.content[0], max_size,
                    "%s CODE=%d %s %s %s",
                    inno_log_header_g[level], code,
                    header1, header2, message);
  }
  if (sz < max_size) {
    sz += 1;
  } else {
    msg.content[max_size -1] = 0;
    sz = max_size;
  }
  msg.size = sz + sizeof(InnoMessage);
  packet.item_size = msg.size;
  packet.common.size = sizeof(InnoDataPacket) + msg.size;
  InnoPacketReader::set_packet_crc32(&packet.common);
  {
    if (message_udp_sender_) {  // cannot lock mutex outside
      std::unique_lock<std::mutex> lk(message_udp_mutex_);
      message_udp_sender_->write(&packet, packet.common.size);
    }
  }

  if (has_ws_()) {
    ws_->write_ws_socket_cpacket(
        cmd_parser_.lidar.lidar_id,
        reinterpret_cast<InnoCommonHeader*>(&packet));
  }

  if (level == INNO_MESSAGE_LEVEL_FATAL) {
    std::unique_lock<std::recursive_mutex> lk(recur_mutex_);
    if (dtc_manager_ && is_log && !assert_failure_called_) {
      assert_failure_called_ = true;
      dtc_manager_->set_assert_fault();
    }
  }
  return;
}

void PCS::log_callback_(enum InnoLogLevel level,
                        const char *header1,
                        const char *header2,
                        const char *msg) {
  {
    // buffer starting log at most 700 lines or 100 KB
    std::unique_lock<std::mutex> lk(mutex_);
    if (starting_log_lines_ < kStartingLogMaxLines &&
        starting_log_offset_ < sizeof(starting_log_) - 1) {
      uint32_t s = sizeof(starting_log_) - 1 - starting_log_offset_;
      int k = snprintf(starting_log_ + starting_log_offset_,
                       s,
                       "%s %s %s %s\n",
                       header1, inno_log_header_g[level],
                       header2, msg);
      if (k < (int32_t)s) {
        starting_log_offset_ += k;
      } else {
        starting_log_offset_ += s - 1;
      }
    }
    starting_log_lines_++;
  }
  write_log_(level, 0, header1, header2, msg, true);
}


//
//
//
int PCS::recorder_callback_(enum InnoRecorderCallbackType type,
                            const char *buffer, int len) {
  if (type == INNO_RECORDER_CALLBACK_TYPE_RAW) {
    if (data_recorder_) {
      return data_recorder_->write(buffer, len);
    }
  } else if (type == INNO_RECORDER_CALLBACK_TYPE_RAW3) {
    if (bad_data_recorder_) {
      return bad_data_recorder_->write(buffer, len);
    }
  } else if (type == INNO_RECORDER_CALLBACK_TYPE_RAW4) {
    const InnoRaw4Packet *raw_packet =
        reinterpret_cast<const InnoRaw4Packet *>(buffer);
    if (raw_packet) {
      size_t written = raw4_send_(raw_packet);

      // network traffic control : nearly 6.4M/s
      if (written > 0) {
        constexpr int time_10ms = 10 * 1000;
        constexpr int traffic_64k = 64 * 1024;
        constexpr int threshold_2k = 2 * 1024;

        int sleep = written / traffic_64k;
        if (written % traffic_64k > threshold_2k) {
          sleep++;
        }

        if (sleep == 0) {
          sleep = 1;
        }
        usleep(sleep * time_10ms);
      }
    }

    // nonzero will disable raw4 callback
    return raw_udp_sender_ ? 0 : -1;
  }
  return 0;
}

//
//
//
int PCS::raw4_send_(const InnoRaw4Packet *raw_packet) {
  std::unique_lock<std::mutex> lk(raw_udp_mutex_);

  if (!raw_udp_sender_) {
    return 0;
  }

  if (raw_packet->buffer_size <= 0) {
    return 0;
  }

  // inno_log_info("raw4 send, field: %d, size: %d.", raw_packet->field_type,
  //               raw_packet->buffer_size);

  constexpr int MAX_BODY_SIZE = 1400;
  // div_t send_div = div(raw_packet->buffer_size, MAX_BODY_SIZE);
  int send_size = raw_packet->buffer_size / MAX_BODY_SIZE;

  size_t send_last_size = raw_packet->buffer_size % MAX_BODY_SIZE;
  if (send_last_size == 0) {
    if (send_size > 0) {
      send_size--;
    }

    send_last_size = MAX_BODY_SIZE;
  }

  Raw4UdpHeader header;
  header.idx = raw_packet->idx;
  header.field_type = raw_packet->field_type;
  header.field_sequence_id = 0;
  header.flag = 0;

  char buffer[Raw4UdpHeader::kHeaderSize]{0};

  //
  size_t written = 0;
  for (int i = 0; i < send_size; i++) {
    header.field_sequence_id = raw_udp_field_idx_;
    InnoDataPacketUtils::raw4_header_to_net(header, buffer,
                                            Raw4UdpHeader::kHeaderSize);

    raw_udp_sender_->write_raw(buffer, Raw4UdpHeader::kHeaderSize,
                               raw_packet->buffer + written, MAX_BODY_SIZE);

    written += MAX_BODY_SIZE;
    raw_udp_field_idx_++;
  }

  // the last
  {
    header.field_sequence_id = raw_udp_field_idx_;
    if (raw_packet->is_field_last) {
      header.set_field_end();
    }
    InnoDataPacketUtils::raw4_header_to_net(header, buffer,
                                            Raw4UdpHeader::kHeaderSize);

    raw_udp_sender_->write_raw(buffer, Raw4UdpHeader::kHeaderSize,
                               raw_packet->buffer + written, send_last_size);

    written += send_last_size;
    raw_udp_field_idx_++;
  }

  // the last
  if (raw_packet->is_field_last) {
    raw_udp_field_idx_ = 0;
  }

  return written;
}


//
//
//
void PCS::message_callback_(uint32_t from_remote,
                            enum InnoMessageLevel level,
                            enum InnoMessageCode code,
                            const char *msg) {
  write_log_(level, code, "", "", msg, false);
  if (code == INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT) {
    return;
  }
  inno_log_print(static_cast<enum InnoLogLevel>(level), true,
                 from_remote ? "[REMOTE]" : "",
                 static_cast<int>(code), "%s", msg);
  if (!from_remote) {
    if (code == INNO_MESSAGE_CODE_TO_NON_WORKING_MODE) {
      inno_log_info("lidar switch to non-working mode. exit...");
      stop();
    } else if (code == INNO_MESSAGE_CODE_OVERHEAT_PROTECTION) {
      std::unique_lock<std::mutex> lk(mutex_);
      faults_update_freq_control_ = false;
    } else if (code == INNO_MESSAGE_CODE_CANNOT_READ) {
      if (lidar_->is_raw_mem()) {
        // xxx todo: exit if restart too often
        inno_log_error("Cannot read. restart lidar source");
        {
          std::unique_lock<std::mutex> lk(mutex_);
          if (shutdown_) {
            return;
          }
        }
        // xxx todo: may still have race issue?
        lidar_->restart_async();
      } else {
        inno_log_info("Cannot read. exit...");
        stop();
      }
    } else if (code == INNO_MESSAGE_CODE_READ_FILE_END) {
      inno_log_info("Reach file end. exit...");
      stop();
    }
  }
}

int PCS::data_callback_(const InnoDataPacket *pkt) {
  {
    // do not print log here
    // log udp sender is in blocking mode, log here will be
    // blocked if log udp sender was blocked.
    std::unique_lock<std::mutex> lk(data_udp_mutex_);
    if (data_udp_sender_) {
      // inno_log_debug("data size=%u", pkt->common.size);
      int write_err_cnt = 0;
      int err = 0;
      while (true) {
        errno = 0;
        ssize_t ret = data_udp_sender_->write(pkt, pkt->common.size, false);
        // save errno and print it after release mutex
        err = errno;
        if (ret != -1) {
          // send success
          break;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          if (++write_err_cnt >= 5) {
            break;
          } else {
            // do nothing
          }
        } else {
          // stop re-trying if other errors occurred
          break;
        }
        usleep(200);
      }
      if (err != 0) {
        // restart pcs or just re-init data_udp_sender
        std::string udp_ip = std::string(data_udp_sender_->get_ip_str());
        uint16_t data_port = data_udp_sender_->get_port();
        lk.unlock();
        setup_udp_(udp_ip, data_port, &data_udp_mutex_, &data_udp_sender_);
        lk.lock();
        if (data_udp_sender_) {
          data_udp_sender_->write(pkt, pkt->common.size, false);
        }
      }
    }
  }
  // save to cframe
  if (has_ws_()) {
    if (ws_->has_ws_socket()) {
      int interval = 1;
      if (frame_capturer_->has_active_capture_job()) {
        interval = 10;
      }
      inno_cframe_header *cframe =
          cframe_converter_->add_data_packet(pkt, interval);
      if (cframe) {
        cframe_received_++;
        ws_->write_ws_socket_cframe(cmd_parser_.lidar.lidar_id, cframe);
      }
    }
    ws_->write_ws_socket_cpacket(
        cmd_parser_.lidar.lidar_id,
        reinterpret_cast<const InnoCommonHeader*>(pkt));
    frame_capturer_->received_data_packet(pkt);
  }

  if (inno_pc_npy_recorder_) {
    inno_pc_npy_recorder_->add_block(pkt);
  }

  if (rosbag_recorder_) {
     rosbag_recorder_->add_block(pkt);
  }

  if (png_recorder_) {
     bool is_saving = png_recorder_->capture(pkt);
     if (is_saving) {
       png_recorder_->save(cmd_parser_.png_filename);

       delete png_recorder_;
       png_recorder_ = nullptr;
     }
  }

  return 0;
}

int PCS::status_callback_(const InnoStatusPacket *pkt) {
  // std::cout << "xxx todo: dummy cframe" << std::endl;
  inno_log_verify(pkt, "pkt");
  {
    std::unique_lock<std::mutex> lk(status_udp_mutex_);
    int ret = 0;
    uint16_t fault_version_old = last_status_packet_.fault_version;
    if (pkt->fault_version != last_status_packet_.fault_version ||
        !faults_update_freq_control_) {
      ret = dtc_manager_->update_dtc_status(faults_update_freq_control_);
      faults_update_freq_control_ = true;
    }

    memcpy(&last_status_packet_, pkt, sizeof(last_status_packet_));
    last_time_sync_type_ = last_status_packet_.common.timestamp_sync_type;
    if (ret == -2) {
      last_status_packet_.fault_version = fault_version_old;
    }

    if (status_udp_sender_) {
      status_udp_sender_->write(pkt, pkt->common.size);
    }
  }
  {
    std::unique_lock<std::mutex> lk(status_local_udp_mutex_);
    if (status_local_udp_sender_) {
      status_local_udp_sender_->write(pkt, pkt->common.size);
    }
  }
  if (has_ws_()) {
    ws_->write_ws_socket_cpacket(
        cmd_parser_.lidar.lidar_id,
        reinterpret_cast<const InnoCommonHeader*>(pkt));
  }
  if (in_long_duration_test_mode_) {
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME , &spec);
    time_t now_sec = spec.tv_sec;
    uint64_t time_diff = now_sec - long_duration_log_last_save_time_;
    if (time_diff >= kLongDurationLogSaveIntervalSec) {
      // write netstat into /mnt/xxx
      FILE *file = fopen(long_duration_test_log_, "a");
      if (file) {
        /* get time */
        struct tm result_time;
        tm *tm_info = localtime_r(&now_sec, &result_time);
        char tbuffer[32];
        strftime(tbuffer, sizeof(tbuffer) - 1, "%Y-%m-%d %H:%M:%S", tm_info);
        tbuffer[sizeof(tbuffer) - 1] = 0;
        /* save log */
        fprintf(file, "%s netstat rx_speed=%huKBps tx_speed=%huKBps "
                      "cpu_percentage=%hu/%hu/%hu/%hu\n",
                      tbuffer,
                      pkt->counters.netstat_rx_speed_kBps,
                      pkt->counters.netstat_tx_speed_kBps,
                      pkt->counters.sys_cpu_percentage[0],
                      pkt->counters.sys_cpu_percentage[1],
                      pkt->counters.sys_cpu_percentage[2],
                      pkt->counters.sys_cpu_percentage[3]);
        fclose(file);
        sync();

        long_duration_log_last_save_time_ = spec.tv_sec;
      }
    }
  }
  return 0;
}

void PCS::set_up_long_duration_test_mode_() {
  in_long_duration_test_mode_ = true;
}

void PCS::clear_long_duration_test_mode_() {
  in_long_duration_test_mode_ = false;
}

bool PCS::is_shutdown_() {
  std::unique_lock<std::mutex> lk(mutex_);
  return shutdown_;
}

void PCS::wait_until_shutdown_() {
  std::unique_lock<std::mutex> lk(mutex_);
  // cond_.wait(lk, std::bind(&PCS::is_shutdown_, this));
  cond_.wait(lk, [this]{return shutdown_;});
}

void PCS::stop() {
  // add the shutdown
  {
    std::unique_lock<std::mutex> lk(mutex_);
    shutdown_ = true;
  }
  cond_.notify_all();
}

void PCS::run() {
  if (has_ws_()) {
    ws_->start();
    inno_log_info("ws server started on port %d", cmd_parser_.tcp_port);
  } else {
    inno_log_info("ws server is disabled");
  }

  if (dtc_manager_) {
    dtc_manager_->start();
    inno_log_info("begins to listen to FW fault");
  }

  bool already_shutdown = false;
  int32_t count = 0;

  // sleep as command line instructed
  for (uint64_t i = 0;
       i < cmd_parser_.sleep_second * 1000;
       i++) {
    if (is_shutdown_()) {
      already_shutdown = true;
      break;
    }
    usleep(1000);
  }

  while (1) {
    if (is_shutdown_()) {
      already_shutdown = true;
      break;
    }
    enum InnoLidarMode mode;
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus ss;
    uint64_t in_transition_mode_ms;
    int ret = lidar_->get_mode_status(&mode,
                                      &pre_mode, &ss,
                                      &in_transition_mode_ms);
    if (ret == 0) {
      // we want to wait until the server is right mode and status
      if (mode != INNO_LIDAR_MODE_NONE &&
          mode != INNO_LIDAR_MODE_SLEEP &&
          mode != INNO_LIDAR_MODE_STANDBY &&
          mode < INNO_LIDAR_MODE_WORK_MAX &&
          ss == INNO_LIDAR_STATUS_NORMAL) {
        inno_log_info("get_mode reach mode=%d pre_mode=%d status=%d",
                      mode, pre_mode, ss);
        break;
      } else {
        if (count % 100 == 1) {
          inno_log_info("get_mode return mode=%d "
                        "pre_mode=%d status=%d %lums",
                        mode, pre_mode, ss, in_transition_mode_ms);
        }
      }
    } else {
      if (count % 100 == 1) {
        inno_log_warning("get_mode return %d", ret);
      }
    }
    count++;
    usleep(100 * 1000);
  }

  if (it_command_test_) {
    it_command_test_->start();
    inno_log_info("command test thread started.");
  }

  if (fw_log_listener_) {
    fw_log_listener_->start();
  }

  if (!already_shutdown) {
    lidar_->start();
    inno_log_info("lidar started");
  }

  wait_until_shutdown_();

  if (!already_shutdown) {
    lidar_->stop();
    inno_log_info("lidar stopped");
  }

  InnoPcFrameCapture *saved = frame_capturer_;
  frame_capturer_ = NULL;
  delete saved;

  if (fw_log_listener_) {
    fw_log_listener_->stop();
    inno_log_info("fw_log_listener_ stopped");
  }


  if (it_command_test_) {
    it_command_test_->shutdown();
    delete it_command_test_;
    it_command_test_ = nullptr;
  }

  if (command_test_) {
    delete command_test_;
    command_test_ = nullptr;
  }

  if (time_sync_udp_listener_) {
    time_sync_udp_listener_->stop();
    delete time_sync_udp_listener_;
    time_sync_udp_listener_ = nullptr;
  }

  if (dtc_manager_) {
    dtc_manager_->stop();
  }

  if (has_ws_()) {
    ws_->stop();
    inno_log_info("ws server stppped");
  }

  inno_log_info("done.");

  return;
}

void PCS::write_log_s(PCS* context,
                 int level, int code,
                 const char *header1,
                 const char *header2,
                 const char *message,
                 bool is_log) {
  PCS* ctx = reinterpret_cast<PCS*>(context);
  ctx->write_log_(level, code, header1, header2, message, is_log);
}

const char *PCS::get_lidar_ip() const {
  return cmd_parser_.lidar.lidar_ip.c_str();
}

uint32_t PCS::get_lidar_port() const {
  return cmd_parser_.lidar.lidar_port;
}

uint16_t PCS::get_time_sync_type() const {
  return last_time_sync_type_;
}

int PCS::send_file(const std::string &item,
                   const std::string &path,
                   const std::string &offset,
                   const std::string &length,
                   void *conn) {
  inno_log_verify(conn, "conn");
  if (item.empty()) {
    // check path, offset, length
    if (path.empty()) {
      return 400;
    }
    size_t os = 0;
    ssize_t len = -1;
    if (!offset.empty()) {
      os = strtoul(offset.c_str(), nullptr, 10);
    }
    if (!length.empty()) {
      len = strtol(length.c_str(), nullptr, 10);
    }
    new FileSender(path, os, len, conn);
  } else {
    // check type
    if (item == "log_snapshot") {
      {
        std::unique_lock<std::mutex> lk(log_snapshot_mutex_);
        if (log_snapshot_status_ != LOG_SNAPSHOT_SUCCESS) {
          inno_log_info("no available log snapshot, current status: %d",
                        log_snapshot_status_);
          return 500;
        }
      }
      std::vector<std::string> files;
      if (InnoUtils::list_file("/mnt", "log_snapshot", &files)) {
        // list file failed
        return 500;
      }
      if (files.empty()) {
        // download from /tmp if there is no snapshot under /mnt
        if (InnoUtils::list_file("/tmp", "log_snapshot", &files)) {
          return 500;
        }
      }
      if (files.size() != 1) {
        inno_log_warning("there are %" PRI_SIZEU " log snapshot file",
                         files.size());
        return 503;
      }
      new FileSender(files[0], 0, -1, conn);
    } else {
      inno_log_error("do not support download %s", item.c_str());
      return 400;
    }
  }
  return 0;
}

}  // namespace innovusion
