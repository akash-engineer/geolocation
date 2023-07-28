/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk_client/stage_client_read.h"

#include <string>
#include <thread>  // NOLINT
#include <vector>

#include "sdk_client/lidar_client.h"
#include "sdk_client/lidar_client_communication.h"
#include "sdk_common/inno_lidar_packet_utils.h"

namespace innovusion {
StageClientRead::StageClientRead(InnoLidarClient *l,
                                 LidarClientCommunication *lm,
                                 bool use_tcp,
                                 uint16_t udp_port,
                                 int max_retry)
    : mutex_()
    , cond_() {
  init_(l);
  inno_log_verify(lm, "lm should nout be NULL");
  lidar_comm_ = lm;
  max_retry_ = max_retry;
  if (use_tcp) {
    source_ = SOURCE_TCP;
  } else {
    source_ = SOURCE_UDP;
  }
  udp_port_ = udp_port;
}

StageClientRead::StageClientRead(InnoLidarClient *l,
                                 const char *filename,
                                 double play_rate,
                                 double play_rate_x,
                                 int max_file_rewind,
                                 int64_t skip) {
  init_(l);
  lidar_comm_ = NULL;
  filename_ = strdup(filename);
  play_rate_ = play_rate;
  play_rate_x_ = play_rate_x;
  start_time_us_ = 0;
  first_data_us_ = 0;
  total_byte_received_ = 0;

  max_file_rewind_ = max_file_rewind;
  skip_ = skip;
  cannot_open_file_ = false;
  reach_file_end_ = false;
  source_ = SOURCE_FILE;
}

StageClientRead::~StageClientRead(void) {
  lidar_comm_ = NULL;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    inno_log_verify(state_ == InnoLidarBase::STATE_INIT,
                    "invalid state=%d",
                    state_);
  }
  if (filename_) {
    free(filename_);
    filename_ = NULL;
  }
}

void StageClientRead::init_(InnoLidarClient *l) {
  state_ = InnoLidarBase::STATE_INIT;
  lidar_ = l;
  udp_port_ = 0;
  use_mreq_ = false;
  filename_ = NULL;
  play_rate_ = 0;
  play_rate_x_ = 0;
  max_file_rewind_ = 0;
  skip_ = 0;
  cannot_open_file_ = false;
  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);
}

const char *StageClientRead::get_name_(void) const {
  return lidar_->get_name();
}

void StageClientRead::start_reading_(void) {
  std::unique_lock<std::mutex> lk(mutex_);
  inno_log_verify(state_ == InnoLidarBase::STATE_INIT,
                  "%s state=%d", get_name_(), state_);
  state_ = InnoLidarBase::STATE_READING;
  cond_.notify_all();
}

void StageClientRead::stop(void) {
  inno_log_info("%s stop", get_name_());
  std::unique_lock<std::mutex> lk(mutex_);
  if (state_ == InnoLidarBase::STATE_INIT) {
    cond_.wait(lk, [this] {
                   inno_log_info("%s wait for state %d", get_name_(), state_);
                   return state_ != InnoLidarBase::STATE_INIT;
                 });
  }
  inno_log_verify(state_ == InnoLidarBase::STATE_READING ||
                  state_ == InnoLidarBase::STATE_STOPPED ||
                  state_ == InnoLidarBase::STATE_STOPPING,
                  "%s state=%d, forget to call start before stop?",
                  get_name_(), state_);
  if (state_ == InnoLidarBase::STATE_READING) {
    state_ = InnoLidarBase::STATE_STOPPING;
  }
  cond_.notify_all();
  cond_.wait(lk, [this] {
                   inno_log_info("%s wait for state %d", get_name_(), state_);
                   return state_ != InnoLidarBase::STATE_STOPPING;
                 });
}

enum InnoLidarBase::State StageClientRead::get_state() {
  std::unique_lock<std::mutex> lk(mutex_);
  enum InnoLidarBase::State ret = state_;

  return ret;
}

void StageClientRead::final_cleanup(void) {
  if (source_ == SOURCE_FILE) {
  } else if (source_ == SOURCE_TCP) {
  } else {
  }

  {
    std::unique_lock<std::mutex> lk(mutex_);
    state_ = InnoLidarBase::STATE_INIT;
  }
  cond_.notify_all();
}

void StageClientRead::print_stats(void) const {
  inno_log_trace("StageRead: %s",
                 "XXX TODO");
}

int StageClientRead::process(void *in_job, void *ctx,
                             bool prefer) {
  // in_job is ignored
  StageClientRead *s = reinterpret_cast<StageClientRead *>(ctx);
  return s->process_job_(in_job);
}

int StageClientRead::process_job_(void *in_job) {
  int ret;
  start_reading_();
  if (source_ == SOURCE_FILE) {
    int play_round = 1;
    inno_log_info("read from file");
    while (1) {
      ret = read_file_();
      if (max_file_rewind_ >= 0) {
        if (play_round >= max_file_rewind_ + 1) {
          break;
        }
      }
      if (stopping_or_stopped_()) {
        break;
      }
      if (cannot_open_file_) {
        break;
      }
      inno_log_info("%s rewind file %s %d/%d",
                    get_name_(), filename_,
                    play_round, max_file_rewind_);
      play_round++;
    }
  } else if (source_ == SOURCE_UDP) {
    inno_log_info("read from udp");
    ret = read_udps_();
  } else if (source_ == SOURCE_TCP) {
    inno_log_info("read from tcp");
    ret = read_tcp_();
  } else {
    inno_log_panic("invalid source %d", source_);
    ret = 1;
  }

  if (ret) {
    inno_log_error("ret is %d", ret);
    send_fatal_message_();
  } else {
    if (reach_file_end_) {
      lidar_->do_message_callback_fmt(INNO_MESSAGE_LEVEL_INFO,
                                      INNO_MESSAGE_CODE_READ_FILE_END,
                                      "%s reach file end", get_name_());
    }
  }

  {
    std::unique_lock<std::mutex> lk(mutex_);
    inno_log_panic_if_not(state_ == InnoLidarBase::STATE_READING ||
                          state_ == InnoLidarBase::STATE_STOPPING,
                          "%s state=%d", get_name_(), state_);
    state_ = InnoLidarBase::STATE_STOPPED;
    inno_log_info("%s reader new state %d", get_name_(), state_);
  }
  cond_.notify_all();
  return ret;
}

void StageClientRead::send_fatal_message_() {
  lidar_->do_message_callback_fmt(INNO_MESSAGE_LEVEL_CRITICAL,
                                  INNO_MESSAGE_CODE_CANNOT_READ,
                                  "cannot read from lidar %s", get_name_());
}

int StageClientRead::bind_udp_port_(uint16_t port) {
  std::vector<InnoUdpOpt> opts;
  if (use_mreq_) {
    opts.emplace_back(IPPROTO_IP, IP_ADD_MEMBERSHIP,
                      &mreq_, sizeof(mreq_),
                      "IP_ADD_MEMBERSHIP");
  }
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;
  opts.emplace_back(SOL_SOCKET, SO_RCVTIMEO,
                    &tv, sizeof(tv),
                    "SO_RCVTIMEO");
  return InnoUdpHelper::bind(port, opts);
}

void StageClientRead::wait_until_stopping_() {
  std::unique_lock<std::mutex> lk(mutex_);
  // cond_.wait(lk, std::bind(&PCS::is_shutdown_, this));
  cond_.wait(lk, [this] {
                   inno_log_info("%s wait for state %d", get_name_(), state_);
                   return state_ == InnoLidarBase::STATE_STOPPING ||
                       state_ == InnoLidarBase::STATE_STOPPED;
                 });
}

int StageClientRead::read_udp_(int32_t port) {
  int fd = bind_udp_port_(port);
  if (fd < 0) {
    // xxx todo
    return -1;
  }

  inno_log_info("recvfrom UDP %d", port);

  void *buff = NULL;
  while (1) {
    enum InnoLidarBase::State saved;
    {
      std::unique_lock<std::mutex> lk(mutex_);
      saved = state_;
    }
    if (saved == InnoLidarBase::STATE_STOPPING ||
        saved == InnoLidarBase::STATE_STOPPED) {
      inno_log_info("stop reading because of stop signal");
      break;
    }

    if (buff == NULL) {
      buff = lidar_->alloc_buffer_(kMaxReadSize);
      inno_log_verify(buff, "out of memory");
    }

    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    int n;

#ifndef __MINGW64__
    while (-1 == (n = recvfrom(fd, buff, kMaxReadSize,
                               MSG_WAITALL, (struct sockaddr *)&cliaddr,
                               &len)) && errno == EINTR) {
    }
#else
    while (-1 == (n = recvfrom(fd, reinterpret_cast<char *>(buff),
                               kMaxReadSize,
                               0, (struct sockaddr *)&cliaddr,
                               &len)) && errno == EINTR) {
    }
#endif
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        inno_log_info("%s", errno == EAGAIN ?  // EAGAIN means timeout
                      "EAGAIN" : "EWOULDBLOCK");
        continue;
      } else {
        inno_log_error_errno("recv port=%d, n=%d, fd=%d", port, n, fd);
      }
    } else {
      if (n >= ssize_t(sizeof(InnoCommonHeader))) {
        union {
          const InnoDataPacket *data_hd;
          const InnoStatusPacket *status_hd;
          InnoCommonHeader *hd;
        };
        hd = reinterpret_cast<InnoCommonHeader *>(buff);
        if ((hd->version.magic_number == kInnoMagicNumberDataPacket &&
             InnoDataPacketUtils::check_data_packet(*data_hd, n)) ||
            (hd->version.magic_number == kInnoMagicNumberStatusPacket &&
             InnoDataPacketUtils::check_status_packet(*status_hd, n))
            ) {
          add_deliver_packet_(hd);
          buff = NULL;
        } else {
          inno_log_warning("bad packet magic=0x%x size=%d",
                           hd->version.magic_number, n);
        }
      } else {
        inno_log_warning("size too small %d", n);
      }
    }
  }  // while (1)

  if (buff) {
    lidar_->free_buffer_(buff);
    buff = NULL;
  }
  close(fd);

  return 0;
}

int StageClientRead::read_udps_() {
  lidar_->before_read_start();

  static const size_t kPortsCount = 3;
  int32_t ports[kPortsCount];
  char ip[64];
  int ret;
  char my_ip[64];
  for (int round = 0; round < 2; round++) {
    ret = lidar_->comm_->get_server_udp_ports_ip(&ports[0],
                                                 &ports[1],
                                                 &ports[2],
                                                 ip,
                                                 sizeof(ip),
                                                 my_ip,
                                                 sizeof(my_ip));
    if (ret != 0) {
      inno_log_error("cannot get server udp ports %d", ret);
      send_fatal_message_();
      return 1;
    } else {
      inno_log_info("read udps: data:%d message:%d status:%d ip=%s my_ip=%s",
                    ports[0], ports[1], ports[2], ip, my_ip);
    }

    if ((ip[0] == 0 || ip[0] == '0') &&
        udp_port_ == 0) {
      inno_log_error("will not set remote udp port/ip "
                     "because udp_port is 0 and server udp is off");
      send_fatal_message_();
      return 1;
    }

    if (round == 0) {
      // only set_server_udp_ports_ip in the first round
      if (udp_port_ != 0) {
        inno_log_info("set_server_udp_ports_ip(%hu)", udp_port_);
        lidar_->comm_->set_server_udp_ports_ip(udp_port_);
      }
    } else {
      // server udp is still off
      if (ip[0] == 0 || ip[0] == '0') {
        inno_log_error("cannot set remote udp port/ip");
        send_fatal_message_();
        return 1;
      }
      if (my_ip[0] != 0) {
        mreq_.imr_multiaddr.s_addr = inet_addr(ip);
        mreq_.imr_interface.s_addr = inet_addr(my_ip);
        use_mreq_ = true;
        inno_log_verify(mreq_.imr_multiaddr.s_addr != INADDR_NONE,
                        "bad ip m-addr %s", ip);
        inno_log_verify(mreq_.imr_interface.s_addr != INADDR_NONE,
                        "bad ip i-addr %s", my_ip);
        inno_log_info("use multicast addr %s on interface %s",
                      ip, my_ip);
      }
    }
  }

  // remove duplicated
  for (size_t i = 0; i < kPortsCount; i++) {
    for (size_t j = 0; j < i; j++) {
      if (ports[i] == ports[j]) {
        ports[i] = 0;
      }
    }
  }

  std::vector<std::thread*> threads;

  for (size_t i = 0; i < kPortsCount; i++) {
    if (ports[i]) {
      threads.push_back(new std::thread([this, ports, i]() {
                                          read_udp_(ports[i]);
                                        }));
    }
  }

  wait_until_stopping_();

  for (auto& th : threads) {
    th->join();
    delete th;
  }
  return 0;
}

int StageClientRead::read_file_() {
  int file_fd = InnoUtils::open_file(filename_, O_RDONLY, 0);
  if (file_fd >= 0) {
    reach_file_end_ = false;
    int ret = keep_reading_fd_(file_fd, true);
    close(file_fd);
    if (ret == -2) {
      // end of file is not really an error
      reach_file_end_ = true;
      ret = 0;
    }
    return ret;
  } else {
    cannot_open_file_ = true;
    return -1;
  }
}

int StageClientRead::read_tcp_() {
  lidar_->before_read_start();

  int file_fd = lidar_comm_->get_connection(0.5,
                                            256 * 1024);
  if (file_fd >= 0) {
    int zero = 0;
    int ret = lidar_comm_->send_command_with_fd(
        file_fd, NULL, &zero,
        "GET /start/ HTTP/1.0\r\n\r\n");
    if (ret == file_fd) {
      ret = keep_reading_fd_(file_fd, false);
      close(file_fd);
      return ret;
    } else {
      inno_log_error("cannot send start %d", ret);
      send_fatal_message_();
      return -2;
    }
  } else {
    inno_log_error("cannot get tcp connection to server");
    send_fatal_message_();
    return -1;
  }
}

bool StageClientRead::stopping_or_stopped_() {
  std::unique_lock<std::mutex> lk(mutex_);
  if (state_ == InnoLidarBase::STATE_STOPPING ||
      state_ == InnoLidarBase::STATE_STOPPED) {
    inno_log_info("stop reading because of stop signal");
    return true;
  } else {
    return false;
  }
}

void StageClientRead::add_deliver_packet_(InnoCommonHeader *header) {
  lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW,
                               reinterpret_cast<const char*>(header),
                               header->size);
  lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_SRC,
                                    1, header->size);
  lidar_->add_deliver_job_(header);
}

int StageClientRead::keep_reading_fd_(int file_fd, bool is_file) {
  InnoDataPacket *data_packet = NULL;
  size_t data_len_max = kMaxReadSize;
  size_t data_len;
  InnoDataPacket *message_packet = NULL;
  size_t message_len_max = kMaxReadSize;
  size_t message_len;
  InnoStatusPacket *status_packet = NULL;
  size_t status_len_max = kMaxReadSize;
  size_t status_len;

  size_t read_so_far = 0;
  size_t data_cnt = 0;
  size_t message_cnt = 0;
  size_t status_cnt = 0;
  InnoTimestampUs latest_data_us = 0;

  start_time_us_ = lidar_->get_monotonic_raw_time_us();
  first_data_us_ = 0;
  total_byte_received_ = 0;

  int ret = 0;
  while (1) {
    if (stopping_or_stopped_()) {
      break;
    }

    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC_RAW, &spec);
    if (data_packet == NULL) {
      data_packet = reinterpret_cast<InnoDataPacket*>(
          lidar_->alloc_buffer_(data_len_max));
      inno_log_verify(data_packet, "out of memory");
    }
    data_len = data_len_max;
    if (message_packet == NULL) {
      message_packet = reinterpret_cast<InnoDataPacket*>(
          lidar_->alloc_buffer_(message_len_max));
      inno_log_verify(message_packet, "out of memory");
    }
    message_len = message_len_max;
    if (status_packet == NULL) {
      status_packet = reinterpret_cast<InnoStatusPacket*>(
          lidar_->alloc_buffer_(status_len_max));
      inno_log_verify(status_packet, "out of memory");
    }
    status_len = status_len_max;

    int r = InnoPacketReader::read_packet(file_fd,
                                          data_packet,
                                          &data_len,
                                          message_packet,
                                          &message_len,
                                          status_packet,
                                          &status_len,
                                          is_file);
    if (r > 0) {
      read_so_far += r;
      if (data_len) {
        data_cnt++;
        if (InnoDataPacketUtils::check_data_packet(*data_packet,
                                                   r)) {
          latest_data_us = data_packet->common.ts_start_us;
          add_deliver_packet_(&data_packet->common);
          data_packet = NULL;
        } else {
          inno_log_warning("receive corruped data_packet");
        }
      } else if (message_len) {
        message_cnt++;
        if (InnoDataPacketUtils::check_data_packet(*message_packet,
                                                   r)) {
          add_deliver_packet_(&message_packet->common);
          message_packet = NULL;
        } else {
          inno_log_warning("receive corruped message_packet");
        }
      } else if (status_len) {
        status_cnt++;
        add_deliver_packet_(&status_packet->common);
        status_packet = NULL;
      } else {
        inno_log_verify(false, "no valid packet");
      }
      read_file_rate_control_(latest_data_us, r);

      inno_log_trace("%s read from %s return %d",
                     get_name_(), filename_, r);
    } else if (source_ == SOURCE_FILE) {
      if (r < 0) {
        inno_log_fatal("%s %s data is corrupted, "
                       "ret=%d read_so_far=%" PRI_SIZEU " "
                       "data_cnt=%" PRI_SIZEU
                       " message_cnt=%" PRI_SIZEU
                       " status_cnt=%" PRI_SIZEU "",
                       get_name_(), filename_, r, read_so_far,
                       data_cnt, message_cnt, status_cnt);
        ret = -1;
        break;
      } else {
        inno_log_verify(r == 0, "impossible %d", r);
        inno_log_info("%s reach end of %s, "
                      "ret=%d read_so_far=%" PRI_SIZEU " "
                      "data_cnt=%" PRI_SIZEU
                      " message_cnt=%" PRI_SIZEU
                      " status_cnt=%" PRI_SIZEU "",
                      get_name_(), filename_, r, read_so_far,
                      data_cnt, message_cnt, status_cnt);
        ret = -2;
        break;
      }
    } else {
      inno_log_verify(source_ == SOURCE_TCP,
                      "invalid source %d", source_);
      inno_log_warning("%s cannot read from connection, "
                       "ret=%d read_so_far=%" PRI_SIZEU " "
                       "data_cnt=%" PRI_SIZEU
                       " message_cnt=%" PRI_SIZEU
                       " status_cnt=%" PRI_SIZEU "",
                       get_name_(), r, read_so_far,
                       data_cnt, message_cnt, status_cnt);
      ret = -1;
      break;
    }
  }  // while loop
  if (data_packet != NULL) {
    lidar_->free_buffer_(data_packet);
  }
  if (message_packet != NULL) {
    lidar_->free_buffer_(message_packet);
  }
  if (status_packet != NULL) {
    lidar_->free_buffer_(status_packet);
  }
  return ret;
}

void StageClientRead::read_file_rate_control_(InnoTimestampUs last_data_us,
                                              int r) {
  int64_t should_elapsed = 0;
  if (play_rate_ > 0) {
    total_byte_received_ += r;
    should_elapsed = total_byte_received_ / play_rate_;
  } else if (play_rate_x_ > 0) {
    if (first_data_us_) {
      // inno_log_trace("last %lu %lu", last_data_us, first_data_us_);
      if (last_data_us > first_data_us_ + 1000000UL * 3600 * 24 * 365 * 1) {
        inno_log_info("rate control reset");
        first_data_us_ = last_data_us;
        start_time_us_ = lidar_->get_monotonic_raw_time_us();
      }
      if (last_data_us > first_data_us_) {
        should_elapsed = (last_data_us - first_data_us_) / play_rate_x_;
      } else {
        return;
      }
    } else {
      first_data_us_ = last_data_us;
      return;
    }
  } else {
    return;
  }
  InnoTimestampUs now_us = lidar_->get_monotonic_raw_time_us();
  int64_t elapsed_us = now_us - start_time_us_;
  if (should_elapsed > elapsed_us && elapsed_us >= 0) {
    // inno_log_trace("usleep %lu", should_elapsed - elapsed_us);
    usleep(should_elapsed - elapsed_us);
  }
  return;
}

}  // namespace innovusion
