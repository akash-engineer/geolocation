/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <netinet/in.h>
#include <arpa/inet.h>
#include <byteswap.h>
#include <vector>

#include "src/utils/inno_lidar_log.h"
#include "pcs/udp_listener.h"
#include "src/utils/utils.h"

namespace innovusion {

// =======================================================
// UdpListener
// =======================================================
void UdpListener::start() {
  fd_ = udp_listener_bind_();
  worker_thread_ = new InnoThread(name_.c_str(), 0, 1,
                                  udp_listen_s, this,
                                  0, nullptr);
  worker_thread_->start();
}

void UdpListener::stop() {
  worker_thread_->shutdown();
}

int UdpListener::udp_listener_bind_() {
  std::vector<InnoUdpOpt> opts;
  if (InnoUdpHelper::is_multicast_ip_addr(ip_)) {
    // use mreq
    struct ip_mreq mreq;
    memset(&mreq, 0, sizeof(mreq));
    //    inet_aton(ip_.c_str(), &mreq.imr_multiaddr);
    mreq.imr_multiaddr.s_addr = inet_addr(ip_.c_str());
    mreq.imr_interface.s_addr = INADDR_ANY;
    opts.emplace_back(IPPROTO_IP, IP_ADD_MEMBERSHIP,
                      &mreq, sizeof(mreq),
                      "IP_ADD_MEMBERSHIP");
  }
  opts.emplace_back(SOL_SOCKET, SO_RCVTIMEO,
                    &timeout_, sizeof(timeout_),
                    "SO_RCVTIMEO");
  int fd = InnoUdpHelper::bind(port_, opts);
  inno_log_verify(fd > 0,
                  "UdpListener %s bind port[%d] failed", name_.c_str(), port_);
  return fd;
}

void *UdpListener::udp_listen_s(void *context) {
  auto *listener = reinterpret_cast<UdpListener *>(context);
  listener->udp_recv_loop_();
  return nullptr;
}

int UdpListener::udp_recv_loop_() {
  if (fd_ < 0) {
    return -1;
  }
  inno_log_info("Start receiving udp packet from port: %d", port_);
  char log_buffer[kUdpMaxMsgSize];
  struct sockaddr_in sender_addr;
  socklen_t sender_addr_len = sizeof(sender_addr);
  while (!worker_thread_->has_shutdown()) {
    int n = -1;
    while (-1 == (n = recvfrom(fd_, log_buffer, sizeof(log_buffer),
                               MSG_WAITALL,
                               (struct sockaddr*)&sender_addr,
                               &sender_addr_len))
           && errno == EINTR) {
    }
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      } else {
        inno_log_error("error receiving udp packet from port: %d", port_);
      }
    } else {
      data_process_(log_buffer, n);
    }
  }
  InnoUtils::close_fd(fd_);
  return 0;
}

// =======================================================
// Udp_log_listener
// =======================================================
UdpLogListener::UdpLogListener(const std::string &name,
                               uint16_t udp_port,
                               timeval timeout,
                               PCS *pcs)
  : UdpListener(name,
                udp_port,
                timeout),
  pcs_(pcs) {
}

UdpLogListener::~UdpLogListener() = default;

void UdpLogListener::data_process_(char *buf_in, uint32_t len) {
  struct firmware_log *fw_log = reinterpret_cast<struct firmware_log *>(buf_in);
  uint32_t n_tmp = (uint32_t)len;

  if (n_tmp >= sizeof(firmware_log) && n_tmp <= kUdpMaxMsgSize - 1) {
    buf_in[len] = 0;
  } else {
    inno_log_error("size of received data invalid");
    return;
  }

  const char *from =
      fw_log->source == 1 ? "[FROM_FW]"
                          : fw_log->source == 2 ? "[FROM_PTP4L]" : "[UNKNOWN]";

  if (fw_log->type != 0) {
    inno_log_error("log_type mismatched with 0(current), drop the log");
    return;
  }

  if (fw_log->level >= INNO_LOG_LEVEL_MAX ||
      fw_log->level < INNO_LOG_LEVEL_FATAL) {
    inno_log_error("log level is out of range, drop the log");
    return;
  } else if (fw_log->level <= INNO_LOG_LEVEL_WARNING && n_tmp == fw_log->size) {
    InnoLogLevel log_level = INNO_LOG_LEVEL_INFO;
    switch (fw_log->level) {
      case INNO_LOG_LEVEL_FATAL:
      case INNO_LOG_LEVEL_CRITICAL:
      case INNO_LOG_LEVEL_ERROR:
        log_level = INNO_LOG_LEVEL_ERROR;
        break;
      case INNO_LOG_LEVEL_WARNING:
        log_level = INNO_LOG_LEVEL_WARNING;
        break;

      case INNO_LOG_LEVEL_TEMP:
      default:
        log_level = INNO_LOG_LEVEL_INFO;
        break;
    }

    inno_log_3th_program(log_level, from, "%s", fw_log->message);
    return;
  }

  if (n_tmp == fw_log->size) {
    bool is_log = true;
    PCS::write_log_s(pcs_, fw_log->level, fw_log->code, from, "",
                     fw_log->message, is_log);
    // for debug purpose only!!!!!!!
    // inno_log_with_level((enum InnoLogLevel)fw_log->level,
    //                   "%s%s", from, fw_log->message);
  } else {
    inno_log_error("received size invalid: %d, drop the log", len);
  }
}

// =======================================================
// TimeSyncListener
// =======================================================
TimeSyncUdpListener::TimeSyncUdpListener(const std::string &name,
                    const std::string &ip_in, uint16_t port_in,
                    timeval timeout,
                    const std::string& ip_out,
                    uint16_t port_out,
                    PCS *pcs)
                    : UdpListener(name,
                                  ip_in,
                                  port_in > 0 ? port_in : kDefaultPortIn,
                                  timeout),
                    feed_back_sender_(ip_out, port_out),
                    pcs_(pcs),
                    last_sync_print_time_(0) {
}

void TimeSyncUdpListener::data_process_(char *buf_in, uint32_t len) {
  // check data
  if (len != sizeof(time_sync_packet) || len >= kUdpMaxMsgSize) {
    inno_log_info("%s received invalid size data %du", name_.c_str(), len);
    return;
  }

  // get current time sync type, if get failed we ignore this check
  uint16_t time_sync_type_v = pcs_->get_time_sync_type();
  if (time_sync_type_v >= static_cast<uint16_t>(INNO_TIME_SYNC_TYPE_MAX)) {
    return;
  }

  auto *data_in = reinterpret_cast<time_sync_packet *>(buf_in);
  memset(data_in->reserved, 0, sizeof(data_in->reserved));

  // fill data with current timestamps
  uint64_t cur_tsc_time = InnoUtils::get_time_ns(CLOCK_MONOTONIC);
  uint64_t cur_sys_time = InnoUtils::get_time_ns(CLOCK_REALTIME);
  // tsc
  data_in->send_tsc_time = bswap_64(cur_tsc_time);
  // sys time
  data_in->send_sys_time = bswap_64(cur_sys_time);
  // ptp time, currently, equal to sys time
  data_in->send_ptp_time = data_in->send_sys_time;
  data_in->sender_id = 7;
  data_in->time_sync_status = static_cast<uint8_t>(time_sync_type_v);
  // feedback
  ssize_t sent_bytes =
      feed_back_sender_.write(data_in, sizeof(time_sync_packet));
  if (sent_bytes != sizeof(time_sync_packet)) {
    inno_log_warning_errno("send time sync packet error, sent size: %zd",
                           sent_bytes);
  }
  // More than 100 seconds to print one time
  inno_log_trace("sent timestamp: %" PRI_SIZEU ", type: %u",
                data_in->send_sys_time,
                data_in->time_sync_status);
  if (cur_sys_time - last_sync_print_time_ > kPrintLogTimeInterval) {
    inno_log_info("sent sys timestamp: %" PRI_SIZEU ", "
                  "tsc timestamp: %" PRI_SIZEU ", "
                  "ptp timestamp:%" PRI_SIZEU ", "
                  "type: %u",
                  cur_sys_time,
                  cur_tsc_time,
                  cur_sys_time,
                  data_in->time_sync_status);
    last_sync_print_time_ = cur_sys_time;
  }
}
}  // namespace innovusion
