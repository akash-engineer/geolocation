/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_STAGE_CLIENT_DELIVER_H_
#define SDK_CLIENT_STAGE_CLIENT_DELIVER_H_

#include <mutex>  // NOLINT
#include <string>

#include "utils/utils.h"
#include "utils/config.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_client/lidar_fault_check.h"

class InnoCommonHeader;

namespace innovusion {
class InnoLidarClient;

class StageClientDeliverConfig: public Config {
 public:
  StageClientDeliverConfig() : Config() {
  }

  virtual const char* get_type() const {
    return "LidarClient_StageClientDeliver";
  }

  int set_key_value_(const std::string &key, double value) override {
    return -1;
  }

  int set_key_value_(const std::string &key, const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  END_CFG_MEMBER()
};

class StageClientDeliver {
  friend InnoLidarClient;

 public:
  explicit StageClientDeliver(InnoLidarClient *l);
  ~StageClientDeliver(void);

 public:
  static int process(void *job, void *ctx, bool prefer);

 public:
  void print_stats(void) const;

 private:
  const char *get_name_() const;
  int process_job_(InnoCommonHeader *pkt,
                   bool prefer);
  int check_lidar_fault_in_client_(const InnoDataPacket &pkt);
  int update_galvo_check_result_(const InnoGalvoCheckResult &check_result);
  int update_max_distance_check_result(
                       const InnoMaxDistanceResult &check_result);

 public:
  static const size_t kMaxXyzDataPacketBufSize = 1024 * 1024;

 private:
  InnoLidarClient *lidar_;
  std::mutex mutex_;

  uint64_t stats_total_jobs_;
  uint64_t stats_dropped_jobs_;
  uint64_t stats_data_jobs_;
  uint64_t stats_message_jobs_;
  uint64_t stats_status_jobs_;
  uint64_t stats_points_;
  uint64_t stats_2nd_return_points_;
  uint64_t stats_frames_;
  InnoMean convert_xyz_mean_ms_;
  InnoMean callback_mean_ms_;

  StageClientDeliverConfig config_base_;
  StageClientDeliverConfig config_;
  union {
    char xyz_data_packet_buf_[kMaxXyzDataPacketBufSize];
    InnoDataPacket xyz_data_packet_;
  };

  bool has_set_galvo_mirror_fault_;
  bool has_set_galvo_sensor_fault_;
  uint64_t check_galvo_times_;
  // 4bit + 4bit + 8bit
  // status + check code + speed (int km/h)
  uint16_t last_galvo_check_status_;
  // angle * 100
  uint16_t last_galvo_check_angle_;

  RingIdConverterInterface *ring_id_converter_{nullptr};
};

}  // namespace innovusion

#endif  // SDK_CLIENT_STAGE_CLIENT_DELIVER_H_

