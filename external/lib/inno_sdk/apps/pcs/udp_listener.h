/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef PCS_UDP_LISTENER_H_
#define PCS_UDP_LISTENER_H_

#include <pthread.h>
#include <mutex>  // NOLINT
#include <condition_variable>  // NOLINT
#include <string>
#include <utility>
#include "sdk_common/lidar_base.h"
#include "pcs/pcs.h"
#include "pcs/udp_sender.h"

namespace innovusion {

class PCS;
/**
 * class UdpListener
 * User can create an UdpListener to receive udp packet from specified port.
 * 1. create
 * 2. start
 * 3. stop
 * 4. data_process_() should be implemented by sub-class
 */
class UdpListener {
 public:
  static constexpr uint16_t kUdpMaxMsgSize =
      65535 - 20 /*ip header*/ - 8 /*udp header*/;

 public:
  UdpListener(std::string name,
              uint16_t udp_port,
              timeval timeout)
  : name_(std::move(name)),
    ip_("0.0.0.0"),
    port_(udp_port),
    timeout_(timeout),
    fd_(-1),
    worker_thread_(nullptr) {
  }
  UdpListener(std::string name,
              std::string ip,
              uint16_t udp_port,
              timeval timeout)
  : name_(std::move(name)),
  ip_(std::move(ip)),
  port_(udp_port),
  timeout_(timeout),
  fd_(-1),
  worker_thread_(nullptr) {
  }
  virtual ~UdpListener() {
    if (!worker_thread_) {
      if (!worker_thread_->has_shutdown()) {
        worker_thread_->shutdown();
      }
      delete worker_thread_;
      worker_thread_ = nullptr;
    }
  }

  static void* udp_listen_s(void* context);
  void set_timeout(timeval timeout) {
    timeout_ = timeout;
  }
  /**
   * start listening, startup work thread
   */
  void start();
  /**
   * stop listening
   */
  void stop();

 private:
  int udp_listener_bind_();
  /**
   * working function of worker thread.
   * listening on port and receive data.
   * call data_process_() to process received data.
   * @return
   */
  int udp_recv_loop_();
  virtual void data_process_(char *buf_in, uint32_t len) = 0;

 protected:
  std::string name_;

 private:
  std::string ip_{""};
  int port_{-1};
  /**
   * default is 500ms, can be set by user
   */
  timeval timeout_{.tv_sec = 0, .tv_usec = 500 * 1000};

  int fd_{-1};
  InnoThread *worker_thread_{nullptr};
};

class UdpLogListener : public UdpListener {
 public:
  UdpLogListener(const std::string &name,
                 uint16_t udp_port,
                 timeval timeout,
                 PCS *pcs);
  ~UdpLogListener() override;

 private:
  void data_process_(char *buf_in, uint32_t len) override;

 private:
  PCS* pcs_;

 private:
  struct firmware_log {
    uint32_t size;          // the total message size, include all fields
    uint32_t source;        // 1 for firmware
    uint32_t type;          // 0 means log
    uint32_t idx;           // increase by one each time, start from 0
    uint32_t level;         // INNO_LOG_LEVEL_FATAL = 0,
                            // INNO_LOG_LEVEL_CRITICAL = 1,
                            // INNO_LOG_LEVEL_ERROR = 2,
                            // INNO_LOG_LEVEL_TEMP = 3,
                            // INNO_LOG_LEVEL_WARNING = 4,
                            // INNO_LOG_LEVEL_DEBUG = 5,
                            // INNO_LOG_LEVEL_INFO = 6,
                            // INNO_LOG_LEVEL_TRACE = 7,
                            // INNO_LOG_LEVEL_DETAIL = 8,
                            // INNO_LOG_LEVEL_MAX = 9,
    uint32_t code;          // set to 0 for now
    uint32_t reserved[2];   // set to all 0 for now
    char message[0];        // variable length, max is 64000
  };
};

class TimeSyncUdpListener : public UdpListener {
 public:
  static const int kDefaultPortIn = 12111;
  constexpr static const char kDefaultIpOut[] = "239.0.0.1";
  static const int kDefaultPortOut = 12117;
  static const uint64_t kPrintLogTimeInterval = 100000000000;  // 100seconds

 public:
  TimeSyncUdpListener(const std::string &name,
                      const std::string &ip_in, uint16_t port_in,
                      timeval timeout,
                      const std::string& ip_out,
                      uint16_t port_out,
                      PCS *pcs);

  ~TimeSyncUdpListener() override = default;

 private:
  void data_process_(char *buf_in, uint32_t len) override;

 private:
  UdpSender feed_back_sender_{kDefaultIpOut, kDefaultPortOut};
  PCS *pcs_{nullptr};
  uint64_t last_sync_print_time_;

 private:
  /**
   * Rx & Tx package structure are same
   * total 96 bytes
   */
  typedef struct __attribute__((packed)) {
    /**
     * Lidar should back fill with received data
     */
    uint32_t master_index;
    /**
     * Lidar should back fill with received data
     */
    uint32_t send_counter;
    /**
     * Lidar tsc time, lidar uptime, Unix timestamp in NS
     */
    uint64_t send_tsc_time;
    /**
     * Lidar current system time, in Unix timestamp, unit is NS
     */
    uint64_t send_sys_time;
    /**
     * Lidar current ptp time, in Unix timestamp, unit is NS
     */
    uint64_t send_ptp_time;
    /**
     * Lidar's node id, always 7
     */
    uint8_t sender_id;
    /**
     * Lidar back fill with current time sync status
     * value: enum InnoTimeSyncType in inno_lidar_packet.h
     */
    uint8_t time_sync_status;
    /**
     * Lidar should back fill will 0x00
     */
    char reserved[62];
  } time_sync_packet;
};
}  // namespace innovusion

#endif  // PCS_UDP_LISTENER_H_
