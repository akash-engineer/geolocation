/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <stdio.h>
#include <stdlib.h>
#include "pcs/dtc_manager.h"
#include "src/utils/utils.h"
#include "src/sdk_common/inno_faults_common.h"
#include "src/thirdparty/nlohmann/json.hpp"

using json = nlohmann::json;

namespace innovusion {

DtcManager::DtcManager(const char* filename, LidarSource* lidar) :
                       filename_(filename) {
  inno_log_verify(lidar, "lidar invalid");
  lidar_ = lidar;
  write_file_ts_ = 0;
  if (lidar_->is_live_direct_memory()) {
    int ret = open_file_and_load_info_();
    // set priority same as status_report currently
    std::string path("/tmp/pcs_socket_unix.skt");
    fw_fault_listener_ = new FwFaultListener(path, 0, 200 * 1000, this);
    inno_log_verify(fw_fault_listener_, "fw_fault_listener_");
    fw_fault_listen_thread_
      = new InnoThread("fw_fault_listener",
                       41,
                       1,
                       FwFaultListener::fw_fault_listen_loop_s_,
                       fw_fault_listener_,
                       0,
                       NULL);
    inno_log_verify(fw_fault_listen_thread_, "fw_fault_listen_thread_");
    if (ret == 0) {
      inno_log_info("Dtc manager create completed");
    } else {
      inno_log_info("open or load dtc error: %d", ret);
    }
    is_lidar_in_live_direct_memory_mode_ = true;
  } else {
    fw_fault_listener_ = nullptr;
    fw_fault_listen_thread_ = nullptr;
    is_lidar_in_live_direct_memory_mode_ = false;
  }
}

DtcManager::~DtcManager() {
  //
  // currently disable history fatuts auto-clear function
  //
  #if 0
  char buffer[kInnoInternalDtcsMaxFileSize] = {'\0'};
  int ret = inno_lidar_get_attribute_string(lidar_->get_lidar_handle(),
                                           "sync_cycle_status",
                                            buffer, sizeof(buffer));
  std::unique_lock<std::mutex> lk(mutex_);
  if (strcmp(buffer, faults_status_last_.c_str()) != 0 && ret == 0) {
    write_dtc_without_lock_(buffer);
    faults_status_last_ = buffer;
  }
  #endif
  if (fw_fault_listen_thread_) {
    delete fw_fault_listen_thread_;
    fw_fault_listen_thread_ = NULL;
  }
  if (fw_fault_listener_) {
    delete fw_fault_listener_;
    fw_fault_listener_ = NULL;
  }
  update_dtc_status(false);
}

int DtcManager::open_file_and_load_info_() {
  // std::unique_lock<std::mutex> lk(mutex_);
  if (filename_.empty()) {
    inno_log_info("no specific dtc_file to load!");
    return -1;
  }
  // open
  int fd = open(filename_.c_str(), O_RDONLY);
  if (fd < 0) {
    inno_log_warning("open %s failed.", filename_.c_str());
    return -2;  // open error
  }
  // read
  lseek(fd, 0, SEEK_SET);
  std::string input_string(kInnoInternalDtcsMaxFileSize + 1, '\0');
  int read_len = -1;
  while (-1 == (read_len = read(fd, &input_string[0],
                                kInnoInternalDtcsMaxFileSize)) &&
                                errno == EINTR) {
  }
  // close
  close(fd);
  if (read_len < 0) {
    inno_log_error("read dtc file error: %d", read_len);
    return -3;
  } else if (read_len == 0) {
    inno_log_info("file %s read size %d, no history faults",
                   filename_.c_str(), read_len);
    return 0;
  } else if (read_len < kMinFaultItemLength) {
    try {
      json input = json::parse(input_string);
      if (input.is_null()) {
        inno_log_info("file %s read finished, no history faults",
                       filename_.c_str());
        return 0;
      }
    } catch (json::exception& e) {
      inno_log_error("parse dtc error %s exception=%s",
                      input_string.c_str(), e.what());
      return -7;
    }
    inno_log_warning("read dtc file error, item size too small %d, %s",
                      read_len, input_string.c_str());
    return 0;
  }

  try {
    inno_log_info("loaded string: %s, len: %d", input_string.c_str(), read_len);

    // consistence check and map to internal dtcs
    json input = json::parse(input_string);
    int counters = 0;
    for (const auto& item : input) {
      if (item.is_null()) {
        continue;
      }
      if (item.find("fid") == item.end() ||
          item.find("name") == item.end() ||
          item.find("fault_status") == item.end() ||
          item.find("no_fault_cycle_so_far") == item.end()) {
        inno_log_error("dtc file format error, please check");
        return -4;
      }
      enum InnoLidarInFault fid = item["fid"].get<enum InnoLidarInFault>();
      // find the matched item
      if (fid >= INNO_LIDAR_IN_FAULT_MAX || fid < INNO_LIDAR_IN_FAULT_OTHER) {
        inno_log_error("fid mismatch, fid: %d", fid);
        return -5;
      }
      std::string name = item["name"].get<std::string>();
      if (strcmp(Inno_faults_def[fid].name, name.c_str()) != 0) {
        inno_log_error("name of fid(%d) mismatch: %s vs. %s, broken file?",
                        fid,
                        name.c_str(),
                        Inno_faults_def[fid].name);
        return -6;
      }
      counters++;
    }  // check passed and mapping finished
    last_fault_status_ = input_string;
    inno_lidar_set_attribute_string(lidar_->get_lidar_handle(),
                                   "init_faults_info", input_string.c_str());
    inno_log_info("load dtcs from %s completed, total %d/%u Dtcs loaded",
                   filename_.c_str(), counters, kMaxInnoInternalDtcs);
  } catch (json::exception& e) {
    inno_log_error("parse dtc error %s exception=%s",
                    input_string.c_str(), e.what());
    return -7;
  }

  return 0;
}

int DtcManager::write_dtc_without_lock_(const std::string str) {
  if (filename_.empty()) {
    inno_log_info("no specific dtc_file to write!");
    return -1;
  }
  std::string filename = filename_ + ".dtc.tmp";
  const char* tmp_filename = filename.c_str();
  int fd = open(tmp_filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    inno_log_error("dfc write error: fd %d", fd);
    return -3;
  }
  int sz = lseek(fd, 0, SEEK_END);
  if (sz != 0) {
    inno_log_error("size error: %d", sz);
    close(fd);
    remove(tmp_filename);
    return -4;
  }
  int str_len = str.size();
  int written = 0;
  while (-1 == (written = write(fd, &str[0], str_len + 1)) &&
        (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
  }
  close(fd);
  if (written != str_len + 1) {
    inno_log_error("dtc temp file write error: %d/%d, errno: %d",
                    written, str_len + 1, errno);
    remove(tmp_filename);
    return -5;
  }
  // confirm
  fd = open(tmp_filename, O_RDONLY);
  if (fd < 0) {
    inno_log_error("cannot reopen file: fd %d", fd);
    return -6;
  }
  lseek(fd, 0, SEEK_SET);
  char read_buf[kInnoInternalDtcsMaxFileSize];
  read_buf[0] = '\0';
  int read_len = read(fd, &read_buf[0], sizeof(read_buf));
  close(fd);
  if (strcmp(read_buf, str.c_str()) != 0) {
    inno_log_error("re-read error: %d vs. %d, content: %s vs. %s",
                    read_len, written, read_buf, str.c_str());
    remove(tmp_filename);
    return -7;
  }
  rename(tmp_filename, filename_.c_str());
  last_fault_status_ = str;
  return 0;
}


int DtcManager::update_dtc_status(bool freq_control) {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return 0;
  }
  uint64_t now_ts = InnoUtils::\
                    get_time_ns(CLOCK_MONOTONIC_RAW) / 1000000.0;
  std::unique_lock<std::recursive_mutex> lk(recur_mutex_);
  bool in_freq_control = now_ts - write_file_ts_ <=
                                  kMinTimeIntervalWriteFaults;
  if (freq_control && in_freq_control && write_file_ts_ != 0) {
    return -2;
  }
  write_file_ts_ = now_ts;
  lk.unlock();
  char buffer[kInnoInternalDtcsMaxFileSize] = {'\0'};
  int ret = inno_lidar_get_attribute_string(lidar_->get_lidar_handle(),
                                           "inner_faults_info",
                                            buffer, sizeof(buffer));
  inno_log_verify(ret == 0, "return value: %d", ret);
  ret = -1;
  const char* file_play_mode = "File play mode";
  const char* client_mode = "PCS_client_mode";
  lk.lock();
  if (strcmp(last_fault_status_.c_str(), buffer) != 0 &&
      strcmp(buffer, client_mode) != 0 &&
      strncmp(buffer, file_play_mode, strlen(file_play_mode)) != 0) {
    ret = write_dtc_without_lock_(buffer);
  }
  return ret;
}

std::string DtcManager::get_inner_faults_all_info() {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return "Not a live direct memory lidar";
  }
  char buffer[kInnoInternalDtcsMaxFileSize] = {'\0'};
  int ret = inno_lidar_get_attribute_string(lidar_->get_lidar_handle(),
                                           "inner_faults_all_info",
                                            buffer, sizeof(buffer));
  if (ret == 0) {
    return buffer;
  } else {
    inno_log_verify(false, "got return value: %d", ret);
  }
}

std::string DtcManager::get_inner_faults_info() {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return "Not a live direct memory lidar";
  }
  char buffer[kInnoInternalDtcsMaxFileSize] = {'\0'};
  int ret = inno_lidar_get_attribute_string(lidar_->get_lidar_handle(),
                                           "inner_faults_info_network",
                                            buffer, sizeof(buffer));
  inno_log_verify(ret == 0, "got return value: %d", ret);
  return buffer;
}


int DtcManager::set_fault_external(enum InnoLidarInFault fid) {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return -1;
  }
  int ret = inno_lidar_set_attribute_string(lidar_->get_lidar_handle(),
                                           "set_faults_external",
                                            std::to_string(fid).c_str());
  return ret;
}

int DtcManager::heal_faults_external(enum InnoLidarInFault fid) {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return -1;
  }
  int ret = inno_lidar_set_attribute_string(lidar_->get_lidar_handle(),
                                           "heal_faults_external",
                                            std::to_string(fid).c_str());
  return ret;
}

void DtcManager::sync_fw_faults() {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return;
  }
  char buffer[256] = {'\0'};
  int ret = inno_lidar_set_attribute_string(lidar_->get_lidar_handle(),
                                           "sync_fw_faults",
                                            buffer);
  inno_log_verify(ret == 0, "sync_fw_faults error: %d", ret);
}

void DtcManager::start() {
  if (fw_fault_listen_thread_) {
    fw_fault_listen_thread_->start();
  }
}

void DtcManager::stop() {
  if (fw_fault_listen_thread_) {
    fw_fault_listen_thread_->shutdown();
  }
}

int DtcManager::set_raw_fault(enum InnoLidarInFault fid,
                              bool cond,
                              bool history_fault) {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return -1;
  }
  inno_log_verify(fid >= INNO_LIDAR_IN_FAULT_OTHER &&
                  fid < INNO_LIDAR_IN_FAULT_MAX,
                 "Fid is invalid %d", fid);
  std::string str("SET_");
  if (history_fault) {
    str += "HISTORY_";
  }
  str += Inno_faults_def[fid].name;
  str += "_";
  str += std::to_string(fid);
  return inno_lidar_set_attribute_string(lidar_->get_lidar_handle(),
                                         str.c_str(),
                                         std::to_string(cond).c_str());
}

void DtcManager::set_assert_fault() {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return;
  }
  set_raw_fault(INNO_LIDAR_IN_FAULT_ASSERT_FAILURE, true);
  update_dtc_status(false);
}

int DtcManager::heal_raw_fault(enum InnoLidarInFault fid, bool cond) {
  if (!is_lidar_in_live_direct_memory_mode_) {
    return -1;
  }
  inno_log_verify(fid >= INNO_LIDAR_IN_FAULT_OTHER &&
                  fid < INNO_LIDAR_IN_FAULT_MAX,
                 "Fid is invalid %d", fid);
  std::string str(Inno_faults_def[fid].name);
  str = "HEAL_" + str + "_";
  str += std::to_string(fid);
  return inno_lidar_set_attribute_string(lidar_->get_lidar_handle(),
                                         str.c_str(),
                                         std::to_string(cond).c_str());
}

FwFaultListener::FwFaultListener(const std::string& filename,
                                 int timeout_s,
                                 int timeout_us,
                                 DtcManager* dtc_manager) {
  inno_log_verify(!filename.empty(), "filename length error");
  inno_log_verify(dtc_manager, "dtc_manager");
  sock_fd_ = -1;
  filename_ = filename;
  tv_.tv_sec = timeout_s;
  tv_.tv_usec = timeout_us;
  dtc_manager_ = dtc_manager;
}

FwFaultListener::~FwFaultListener() {
  if (sock_fd_ != -1) {
    close(sock_fd_);
    sock_fd_ = -1;
  }
  dtc_manager_ = NULL;
}

void FwFaultListener::fw_listener_loop_() {
  inno_log_verify(sock_fd_ == -1, "sock_fd_ not initialized");
  sock_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
  inno_log_verify(sock_fd_ >= 0, "create unix domian socket failed");
  unlink(filename_.c_str());
  struct sockaddr_un un;
  memset(&un, 0, sizeof(un));
  un.sun_family = AF_UNIX;
  int ret = snprintf(un.sun_path,
                     sizeof(un.sun_path),
                     "%s", filename_.c_str());
  inno_log_verify(ret <= static_cast<int>(sizeof(un.sun_path)),
                  "snprintf sun_path: %d", ret);
  int bind_ret = bind(sock_fd_, (const sockaddr*)&un, sizeof(un));
  inno_log_verify(bind_ret >= 0, "fail to bind: %d", bind_ret);

  int set_socket_opt = setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv_,
                                  sizeof(tv_));
  inno_log_verify(set_socket_opt >= 0,
                  "failed to set receive timeout value: %d", set_socket_opt);
  inno_log_info("UNIX domain socket initialized OK");

  while (!dtc_manager_->fw_fault_listen_thread_->has_shutdown()) {
    int len = 0;
    char buffer[512];
    while (-1 == (len = recv(sock_fd_, buffer,
                             sizeof(buffer), 0)) &&
                                 errno == EINTR) {
    }
    if (len < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // do nothing and retry
      } else {
        inno_log_error_errno("error receiving from unix domain socket: %d",
                                                                      len);
      }
    } else {
      FwSocketPacket* fw_fault = reinterpret_cast<FwSocketPacket*>(buffer);
      FaultMessage* fw_msg = fw_fault->fault_msg;
      bool check_fail = fw_fault->type != INNO_FW_SOCKET_IPC_TYPE_FAULT ||
                        fw_fault->size != static_cast<uint32_t>(len) ||
                        fw_fault->size != sizeof(struct FwSocketPacket) +
                                          sizeof(struct FaultMessage) ||
                        fw_msg->fault_id >= INNO_LIDAR_IN_FAULT_MAX ||
                        fw_msg->fault_id < INNO_LIDAR_IN_FAULT_OTHER ||
                        fw_msg->mode < INNO_FAULT_SET_FALSE ||
                        fw_msg->mode >= INNO_FAULT_UDS_OPERATION_MAX;
      if (check_fail) {
        inno_log_error("length of received data len invalid");
      } else {
        enum InnoLidarInFault fid = InnoLidarInFault(fw_msg->fault_id);
        if (fw_msg->mode == INNO_FAULT_SET_FALSE ||
            fw_msg->mode == INNO_FAULT_SET_TRUE) {
          dtc_manager_->set_raw_fault(fid,
                        fw_msg->mode == INNO_FAULT_SET_TRUE);
        } else if (fw_msg->mode == INNO_FAULT_SET_HISTORY) {
          dtc_manager_->set_raw_fault(fid, true, true);
        } else {
          dtc_manager_->heal_raw_fault(fid,
                        fw_msg->mode == INNO_FAULT_HEAL_TRUE);
        }
      }
    }
    // read the IPC to sync FW faults
    dtc_manager_->sync_fw_faults();
    dtc_manager_->update_dtc_status();
  }
  close(sock_fd_);
  sock_fd_ = -1;
}  // while (1)

}  // namespace innovusion
