/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/lidar_source.h"

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <thread>  // NOLINT

#include "src/sdk_common/inno_lidar_api.h"
#include "src/sdk_common/inno_lidar_other_api.h"
#include "src/utils/log.h"
#include "src/utils/utils.h"

namespace innovusion {
LidarSource::LidarSource(const LidarCommandConfig &config,
                         InnoMessageCallback message_callback,
                         InnoDataPacketCallback data_callback,
                         InnoStatusPacketCallback status_callback,
                         void *callback_context)
    : config_(config)
    , message_callback_(message_callback)
    , data_callback_(data_callback)
    , status_callback_(status_callback)
    , callback_context_(callback_context)
    , handle_(-1)
    , id_(0)
    , started_(false) {
  sn_[0] = 0;
  init_();
}

LidarSource::~LidarSource() {
  close_();
}

void LidarSource::close_() {
  if (handle_ >= 0) {
    int ret = inno_lidar_close(handle_);
    inno_log_verify(ret == 0, "close");
    handle_ = -1;
  }
}

void LidarSource::init_() {
  id_ = config_.lidar_id;
  memset(sn_, 0, sizeof(sn_));

  if (config_.lidar_ip.empty()) {
    handle_ = inno_lidar_open_file("pcsf",
                                   config_.data_filename.c_str(),
                                   !config_.processed,
                                   config_.file_speed,
                                   config_.file_rewind,
                                   config_.file_skip * 1000000UL);
    protocol_ = config_.processed ? INNO_LIDAR_PROTOCOL_PCS_FILE :
                INNO_LIDAR_PROTOCOL_RAW_FILE;
  } else {
    const char *ip_str = config_.lidar_ip.c_str();
    uint16_t udp_port = 0;
    if (config_.lidar_udp_port >= 0) {
      if (!config_.processed) {
        inno_log_info("raw_udp not supported, "
                      "force to use pcs_udp");
      }
      protocol_ = INNO_LIDAR_PROTOCOL_PCS_UDP;
      udp_port = config_.lidar_udp_port;
    } else if (config_.processed) {
      protocol_ = INNO_LIDAR_PROTOCOL_PCS_TCP;
    } else {
      if (config_.lidar_ip == "local") {
        ip_str = "127.0.0.1";
        protocol_ = INNO_LIDAR_PROTOCOL_RAW_MEM;
      } else {
        protocol_ = INNO_LIDAR_PROTOCOL_RAW_TCP;
      }
    }
    handle_ = inno_lidar_open_live("pcsl",
                                   ip_str,
                                   config_.lidar_port,
                                   protocol_, udp_port);
  }
  inno_log_verify(handle_ > 0, "no handle");

  int ret = 0;

  if (config_.use_xyz == 1) {
    ret = inno_lidar_set_attribute_string(handle_,
                                          "force_xyz_pointcloud", "1");
    if (ret != 0) {
      inno_log_error("set force_xyz_pointcloud return %d", ret);
    }
  }
  bool is_server = is_pc_server();
  bool need_set = false;
  InnoReflectanceMode reflectance = config_.reflectance;
  if (is_server) {
    std::string reflectance_mode;
    ret = get_attribute("reflectance_mode", &reflectance_mode);
    if (ret == 0) {
      int reflect_tmp = std::stoi(reflectance_mode);
      if (reflect_tmp > INNO_REFLECTANCE_MODE_NONE &&
          reflect_tmp < INNO_REFLECTANCE_MODE_MAX) {
        reflectance = InnoReflectanceMode(reflect_tmp);
        inno_log_info("use reflectance_mode %d from fw", reflect_tmp);
      } else {
        need_set = true;
        inno_log_error("got invalid value: %d", reflect_tmp);
      }
    } else {
      need_set = true;
      inno_log_error("fail to query reflectance_mode from fw: %d. "
                      "use default %d", ret, reflectance);
    }
  } else {
    need_set = true;
  }
  if (need_set) {
    ret = set_reflectance_mode(reflectance);
    if (ret != 0) {
      inno_log_error("get_reflectance return %d", ret);
    }
  }

  InnoMultipleReturnMode muti_return = config_.multireturn;
  if (is_server) {
    std::string multi_return_mode;
    ret = get_attribute("multiple_return", &multi_return_mode);
    if (ret == 0) {
      int muti_return_tmp = std::stoi(multi_return_mode);
      if (muti_return_tmp > INNO_MULTIPLE_RETURN_MODE_NONE &&
          muti_return_tmp < INNO_MULTIPLE_RETURN_MODE_MAX) {
        muti_return = InnoMultipleReturnMode(muti_return_tmp);
        inno_log_info("use mutiple_return_mode %d from fw", muti_return_tmp);
      } else {
        inno_log_error("got invalid value: %d", muti_return_tmp);
      }
    } else {
      inno_log_error("fail to query multiple_return from fw: %d. "
                     "use default %d", ret, muti_return);
    }
  }
  // we should call set return mode because we may write regs
  ret = set_return_mode(muti_return);
  if (ret != 0) {
    inno_log_error("set_multi_return %d", ret);
  }

  if (config_.set_falcon_eye) {
    ret = set_roi(config_.roi_center_h, config_.roi_center_v);
    if (ret != 0) {
      inno_log_error("set_roi %d", ret);
    }
  }

  ret = inno_lidar_get_sn(handle_, sn_, sizeof(sn_));
  if (ret != 0) {
    inno_log_error("get_sn %d", ret);
  }

  ret = inno_lidar_set_parameters(handle_,
                                  "",
                                  config_.yaml_filename.c_str());
  inno_log_verify(ret == 0, "set_parameters %d", ret);

  ret = inno_lidar_set_callbacks(handle_,
                                 message_callback_,
                                 data_callback_,
                                 status_callback_,
                                 NULL,
                                 callback_context_);
  inno_log_verify(ret == 0, "set_callbacks %d", ret);
}

void LidarSource::restart_async() {
  std::thread t1(&LidarSource::restart, this);
  t1.detach();
  inno_log_info("restart_async lidar source");
}

void LidarSource::start() {
  std::unique_lock<std::mutex> lk(mutex_);
  if (handle_ < 0) {
    init_();
  }
  if (!started_) {
    int ret = inno_lidar_start(handle_);
    inno_log_verify(ret == 0, "start");
    started_ = true;
  }
}

void LidarSource::stop() {
  std::unique_lock<std::mutex> lk(mutex_);
  if (started_) {
    int ret = inno_lidar_stop(handle_);
    inno_log_verify(ret == 0, "stop");
    started_ = false;
  }
}

void LidarSource::restart() {
  inno_log_info("restart lidar source");
  stop();
  start();
}

void LidarSource::show_lidar_info() {
  char buf[1024];
  int ret;
  ret = inno_lidar_get_fw_version(handle_, buf, sizeof(buf));
  if (ret == 0) {
    inno_log_info("FW: %s", buf);
  }
  ret = inno_lidar_get_sn(handle_, buf, sizeof(buf));
  if (ret == 0) {
    inno_log_info("SN: %s", buf);
  }
}

int LidarSource::set_reflectance_mode(InnoReflectanceMode reflectance) {
  int ret = inno_lidar_set_reflectance_mode(handle_,
                                            reflectance);
  if (ret != 0) {
    inno_log_warning("set_reflectance %d return %d",
                     reflectance, ret);
  }
  return ret;
}

int LidarSource::set_return_mode(InnoMultipleReturnMode multireturn) {
  int ret = inno_lidar_set_return_mode(handle_,
                                       multireturn);
  if (ret != 0) {
    inno_log_warning("set_return_mode %d return %d",
                     multireturn, ret);
  }
  return ret;
}

int LidarSource::set_roi(double horz_angle, double vert_angle) {
  int ret = inno_lidar_set_roi(handle_, horz_angle, vert_angle);
  if (ret != 0) {
    inno_log_warning("set_roi %f %f return %d",
                     horz_angle, vert_angle, ret);
  }
  return ret;
}

int LidarSource::get_attribute(const std::string &cmd, std::string *result) {
  char buffer[1024 * 64];
  int buffer_len = sizeof(buffer);
  int ret = -1;
  if (cmd == "fw_version") {
    ret = inno_lidar_get_fw_version(handle_, buffer, buffer_len);
  } else if (cmd == "sn") {
    ret = inno_lidar_get_sn(handle_, buffer, buffer_len);
  } else if (cmd == "model") {
    ret = inno_lidar_get_model(handle_, buffer, buffer_len);
  } else if (cmd == "mode_status" || cmd == "mode") {
    enum InnoLidarMode mode;
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    uint64_t in_transition_mode_ms;
    ret = inno_lidar_get_mode_status(handle_, &mode,
                                     &pre_mode, &status,
                                     &in_transition_mode_ms);
    if (ret == 0) {
      snprintf(buffer, sizeof(buffer), "%d,%d,%d,%lu",
               static_cast<int>(mode), static_cast<int>(pre_mode),
               static_cast<int>(status),
               in_transition_mode_ms);
    } else {
      inno_log_warning("cannot get_mode_status %d", ret);
    }
  } else if (cmd == "roi") {
    double h_roi, v_roi;
    ret = inno_lidar_get_roi(handle_, &h_roi, &v_roi);
    if (ret == 0) {
      snprintf(buffer, sizeof(buffer), "%f,%f",
               h_roi, v_roi);
    } else {
      inno_log_warning("cannot get_roi %d", ret);
    }
  } else {
    char buffer_small[8192];
    ret = inno_lidar_get_attribute_string(handle_, cmd.c_str(),
                                          buffer_small, sizeof(buffer_small));
    if (ret == 0) {
      *result += buffer_small;
      return 0;
    } else {
      inno_log_warning("cannot get attribute %s", cmd.c_str());
      return -1;
    }
  }
  if (ret == 0) {
    *result += buffer;
    return 0;
  } else {
    inno_log_warning("get %s failed", cmd.c_str());
    return -1;
  }
}

int LidarSource::set_lidar(const std::string &name, const std::string &value) {
  int val;
  double h, v;
  int ret;
  if (name == "return_mode" || name == "multiple_return") {
    if (!InnoUtils::is_unsinged_decimal_integer(value) ||
        sscanf(value.c_str(), "%d", &val) != 1) {
      inno_log_warning("Invalid query %s %s", name.c_str(), value.c_str());
      return -1;
    } else {
      ret = set_return_mode((InnoMultipleReturnMode)val);
      return ret;
    }
  } else if (name == "reflectance_mode") {
    if (!InnoUtils::is_unsinged_decimal_integer(value) ||
        sscanf(value.c_str(), "%d", &val) != 1) {
      inno_log_warning("Invalid query %s %s", name.c_str(), value.c_str());
      return -1;
    } else {
      ret = set_reflectance_mode((InnoReflectanceMode)val);
      return ret;
    }
  } else if (name == "roi") {
    if (sscanf(value.c_str(), "%lf,%lf", &h, &v) != 2) {
      inno_log_warning("Invalid query %s %s", name.c_str(), value.c_str());
      return -1;
    } else {
      ret = set_roi(h, v);
      return ret;
    }
  } else if (name == "mode") {
    if (!InnoUtils::is_unsinged_decimal_integer(value) ||
        sscanf(value.c_str(), "%d", &val) != 1 ||
        val <= INNO_LIDAR_MODE_NONE ||
        val >= INNO_LIDAR_MODE_WORK_MAX ||
        val == INNO_LIDAR_MODE_SLEEP ||
        val == INNO_LIDAR_MODE_WORK_SHORT_RANGE) {
      inno_log_warning("Invalid query %s %s", name.c_str(), value.c_str());
      return -1;
    } else {
      enum InnoLidarMode pre_mode;
      enum InnoLidarStatus status;
      ret = inno_lidar_set_mode(handle_, InnoLidarMode(val),
                                    &pre_mode, &status);
      if (ret == 0) {
        inno_log_info("set_mode to %d, pre: %d status: %d", val,
                      static_cast<int>(pre_mode),
                      static_cast<int>(status));
      } else {
        inno_log_warning("set_mode to %d failed, ret=%d",
                         val, ret);
      }
      return ret;
    }
  } else {
    /*  name    |  value
     * ----------------------------------
     * enabled  | ...
     * command  | get_udp_raw_data
     */
    ret = inno_lidar_set_attribute_string(handle_, name.c_str(),
                                          value.c_str());
    if (ret != 0) {
      inno_log_warning("set_attribute %s to %s failed, ret=%d",
                       name.c_str(), value.c_str(), ret);
    }
    return ret;
  }
}

int LidarSource::set_recorder_callback(enum InnoRecorderCallbackType type,
                                       InnoRecorderCallback callback,
                                       void *ctx) {
  return inno_lidar_set_recorder_callback(handle_, type, callback, ctx);
}

int LidarSource::set_config_name_value(const std::string &key,
                                       const std::string &value) {
  return inno_lidar_set_config_name_value(handle_, key.c_str(),
                                          value.c_str());
}

bool LidarSource::is_live_direct_memory() {
  std::string is_live_direct_memory_lidar = "";
  if (get_attribute("is_live_direct_memory",
                    &is_live_direct_memory_lidar) != 0) {
    return false;
  }
  return is_live_direct_memory_lidar == "1";
}

bool LidarSource::is_live_lidar() {
  std::string res;
  int ret = get_attribute("is_live_lidar", &res);
  return ret == 0 && res == "yes";
}

bool LidarSource::is_pc_server() {
  std::string res;
  int ret = get_attribute("is_pc_server", &res);
  return ret == 0 && res == "server";
}

}  // namespace innovusion
