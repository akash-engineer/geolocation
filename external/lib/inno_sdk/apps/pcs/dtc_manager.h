/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef PCS_DTC_MANAGER_H_
#define PCS_DTC_MANAGER_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <mutex>  //NOLINT

#include <limits>

#include "pcs/command_parser.h"
#include "pcs/lidar_source.h"
#include "src/utils/inno_thread.h"
#include "src/utils/inno_lidar_log.h"
#include "src/sdk_common/inno_lidar_packet.h"

namespace innovusion {
class DtcManager;

class FwFaultListener {
 private:
  enum FaultMode {
    INNO_FAULT_SET_FALSE = 0,
    INNO_FAULT_SET_TRUE = 1,
    INNO_FAULT_HEAL_FALSE = 2,
    INNO_FAULT_HEAL_TRUE = 3,
    INNO_FAULT_SET_HISTORY = 4,
    INNO_FAULT_UDS_OPERATION_MAX
  };

  enum FwSocketPacketType {
    INNO_FW_SOCKET_IPC_TYPE_MESSAGE = 0,
    INNO_FW_SOCKET_IPC_TYPE_FAULT = 1,
    INNO_FW_SOCKET_IPC_TYPE_MAX
  };

  struct __attribute__((packed)) FaultMessage {
    /********************************
     * mode:
     * 0: fault set false
     * 1: fault set true
     * 2: fault heal false
     * 3: fault set true
     * others: reserve
     ********************************/
    uint16_t mode;

    /********************************
     * fault_id:
     * id of InnoLidarInFault
     * from firmware
     ********************************/
    uint16_t fault_id;
  };

  struct __attribute__((packed)) FwSocketPacket {
    /*********************************
     * type:
     * 0: message
     * 1: firmware fault transport
     * others: reserve
     ********************************/
    enum FwSocketPacketType type;

    /********************************
     * size:
     * total message size,
     * include all fields
     ********************************/
    uint32_t size;

    /*********************************
     * reserve field
     ********************************/
    uint32_t reserve[4];
    union {
      char message[0];
      struct FaultMessage fault_msg[0];
    };
  };

 public:
  FwFaultListener(const std::string& filename,
                  int timeout_s,
                  int timeout_us,
                  DtcManager* dtc_manager);
  ~FwFaultListener();

 public:
  static void* fw_fault_listen_loop_s_(void* context) {
    inno_log_verify(context, "context");
    FwFaultListener* listener = reinterpret_cast<FwFaultListener*>(context);
    inno_log_info("fw_fault_listen_loop begins");
    listener->fw_listener_loop_();
    return NULL;
  }

 private:
  void fw_listener_loop_();

 private:
  int sock_fd_;
  std::string filename_;
  struct timeval tv_;
  DtcManager* dtc_manager_;
};

class DtcManager {
  friend FwFaultListener;

 public:
  DtcManager(const char* filename, LidarSource* lidar);
  ~DtcManager();

 private:
  static const uint32_t kMaxInnoInternalDtcs = 64;
  static const uint32_t kInnoInternalDtcsMaxFileSize = 16 * 1024;
  static const uint32_t kMinTimeIntervalWriteFaults = 5 * 1000;  // 5s
  static const int32_t kMinFaultItemLength = 45;

 public:
  int update_dtc_status(bool freq_control = true);
  std::string get_inner_faults_all_info();
  std::string get_inner_faults_info();
  int set_fault_external(enum InnoLidarInFault fid =\
                              INNO_LIDAR_IN_FAULT_MAX);
  int heal_faults_external(enum InnoLidarInFault fid =\
                                INNO_LIDAR_IN_FAULT_MAX);
  void sync_fw_faults();

 public:
  void start();
  void stop();
  int set_raw_fault(enum InnoLidarInFault fid,
                    bool cond,
                    bool history_fault = false);
  int heal_raw_fault(enum InnoLidarInFault fid, bool cond);
  void set_assert_fault();

 private:
  int open_file_and_load_info_();
  int write_dtc_without_lock_(const std::string str);

 private:
  std::string filename_;
  LidarSource* lidar_;
  std::recursive_mutex recur_mutex_;
  std::string last_fault_status_;

  InnoThread* fw_fault_listen_thread_;
  FwFaultListener* fw_fault_listener_;

  uint64_t write_file_ts_;
  bool is_lidar_in_live_direct_memory_mode_{false};
};
}  // namespace innovusion

#endif  // PCS_DTC_MANAGER_H_
