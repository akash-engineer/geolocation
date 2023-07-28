/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_STAGE_CLIENT_READ_H_
#define SDK_CLIENT_STAGE_CLIENT_READ_H_

#ifndef __MINGW64__
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <condition_variable>  // NOLINT
#include <mutex>               // NOLINT
#include <string>

#include "sdk_common/lidar_base.h"
#include "utils/config.h"
#include "utils/types_consts.h"

namespace innovusion {
class InnoLidarClient;
class LidarClientCommunication;

class StageClientReadConfig: public Config {
 public:
  StageClientReadConfig() : Config() {
    test = 0;
  }

  virtual const char* get_type() const {
    return "LidarClient_StageClientRead";
  }

  int set_key_value_(const std::string &key,
                             double value) override {
    SET_CFG(test);
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  double test;
  END_CFG_MEMBER()
};

class StageClientRead {
  friend InnoLidarClient;

 private:
  enum Source {
    SOURCE_NO = 0,
    SOURCE_FILE,
    SOURCE_TCP,
    SOURCE_UDP,
    SOURCE_MAX,
  };

 public:
  static int process(void *job, void *ctx, bool prefer);

 public:
  StageClientRead(InnoLidarClient *l,
                  LidarClientCommunication *lm,
                  bool use_tcp,
                  uint16_t udp_port,
                  int max_retry);
  StageClientRead(InnoLidarClient *l,
                  const char *filename,
                  double play_rate,
                  double play_rate_x,
                  int rewind,
                  int64_t skip);
  ~StageClientRead(void);

 public:
  void stop(void);
  void final_cleanup(void);
  enum InnoLidarBase::State get_state();
  void print_stats(void) const;

 private:
  void init_(InnoLidarClient *l);
  const char *get_name_(void) const;

  void start_reading_(void);
  int process_job_(void *in_job);
  void send_fatal_message_();
  int bind_udp_port_(uint16_t port);
  void wait_until_stopping_();
  int read_udp_(int32_t port);
  int read_udps_();
  int read_tcp_();
  int read_file_();
  bool stopping_or_stopped_();
  void add_deliver_packet_(InnoCommonHeader *header);
  int keep_reading_fd_(int fd, bool is_file);
  void read_file_rate_control_(InnoTimestampUs last_data_us,
                               int r);

 public:
  static const size_t kMaxReadSize = 65536;

 private:
  InnoLidarClient *lidar_;

  StageClientReadConfig config_base_;
  StageClientReadConfig config_;

  enum Source source_;
  uint16_t udp_port_;
  LidarClientCommunication *lidar_comm_;
  int max_retry_;

  struct ip_mreq mreq_;
  bool use_mreq_;

  char *filename_;
  double play_rate_;
  double play_rate_x_;
  int max_file_rewind_;
  int64_t skip_;
  bool cannot_open_file_;
  bool reach_file_end_;
  size_t total_byte_received_;
  InnoTimestampUs start_time_us_;
  InnoTimestampUs first_data_us_;

  enum InnoLidarBase::State state_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

}  // namespace innovusion

#endif  // SDK_CLIENT_STAGE_CLIENT_READ_H_

