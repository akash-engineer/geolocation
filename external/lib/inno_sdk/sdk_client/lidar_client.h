/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_LIDAR_CLIENT_H_
#define SDK_CLIENT_LIDAR_CLIENT_H_

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

#include <cstddef>
#include <map>
#include <mutex>  // NOLINT
#include <string>

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/lidar_base.h"
#include "sdk_common/resource_stats.h"
#include "utils/config.h"
#include "utils/log.h"
#include "sdk_client/raw_recorder.h"
#include "sdk_client/lidar_fault_check.h"
#include "utils/inno_thread.h"

namespace innovusion {
class ClientStats;
class ConsumerProducer;
class LidarClientCommunication;
class MemPool;
class StageClientRead;
class StageClientDeliver;
class RawReceiver;
class RingIdConverter;

class InnoLidarClient : public InnoLidarBase {
friend class StageClientRead;
friend class StageClientDeliver;
friend class ClientStats;
friend class RawReceiver;

 private:
  enum LidarSource {
    LIDAR_SOURCE_NONE = 0,
    LIDAR_SOURCE_FILE,
    LIDAR_SOURCE_LIVE,
    LIDAR_SOURCE_MAX,
  };

 public:
  // xxx todo: pick the right value
  static const size_t kSignalJobPoolSize = 15;
  static const size_t kAngleJobPoolSize = 20;
  static const size_t kDeliverMessageMaxSize = 60000;
  static const size_t kDeliverPointsMaxBlockNumber = 10000;
  static const size_t kDeliverMessageJobPoolSize = 10;
  static const size_t kDeliverPointsJobPoolSize = 10;
  static const size_t kDeliverStatusJobPoolSize = 10;
  static const size_t kMaxPacketSize = 65536;
  static const size_t kPacketPoolSize = 500;

 public:  // static methods
  static int reader_func(void *job, void *ctx, bool prefer);

 public:
  InnoLidarClient(const char *name, const char *lidar_ip,
                  uint16_t port, bool use_tcp, uint16_t udp_port);
  InnoLidarClient(const char *name, const char *filename, int play_rate,
                  int rewind, int64_t skip);
  ~InnoLidarClient();

 public:
  int read_ps_reg(uint16_t off, uint32_t *value) { return 1;}
  int read_pl_reg(uint16_t off, uint32_t *value) { return 1;}
  int write_ps_reg(uint16_t off, uint32_t value) { return 1;}
  int write_pl_reg(uint16_t off, uint32_t value) { return 1;}

  int set_params_file(const char *lidar_model,
                      const char *yaml_filename) { return 0;}
  int set_config_name_value(const char *name,
                            const char *value);
  int set_reflectance_mode(enum InnoReflectanceMode);
  int set_return_mode(enum InnoMultipleReturnMode ret_mode);
  int set_roi(double h_angle, double v_angle);
  int get_roi(double *h_angle, double *v_angle);
  int set_mode(enum InnoLidarMode mode,
               enum InnoLidarMode *mode_before_change,
               enum InnoLidarStatus *status_before_change);
  int get_mode_status(enum InnoLidarMode *mode,
                      enum InnoLidarMode *pre_mode,
                      enum InnoLidarStatus *status,
                      uint64_t *in_transition_mode_ms);
  int get_attribute(const char *attribute, double *value);
  int get_attribute_string(const char *attribute,
                           char *buf, size_t buf_size);
  int set_attribute_string(const char *attribute,
                           const char *buf);
  int set_faults_save_raw(uint64_t value);
  int set_motion_compensation(double velocity[3],
                              double angular_velocity[3]);
  int thread_setaffinity_np(size_t cpusetsize,
                            const cpu_set_t *cpuset,
                            int exclude_callback_thread);
  int get_fw_state(enum InnoLidarState *state, int *error_code);
  int get_fw_version(char *buffer, int buffer_len);
  int get_sn(char *buffer, int buffer_len);
  int get_model(char *buffer, int buffer_len);
  void start();
  void stop();
  void print_stats(void);
  int before_read_start(void);
  void stats_update_packet_bytes(enum ResourceStats::PacketType type,
                                 size_t packet, size_t byte);
  InnoFrameCheckProcess get_check_galvo_mirror_result(
                                    const InnoDataPacket &pkt,
                                    InnoGalvoCheckResult *check_result);
  int get_check_max_distance_result(const char *buf,
                                    InnoMaxDistanceResult *check_result);
  int check_max_distance_on_tunnel(const InnoDataPacket &pkt);
  int set_galvo_sensor_fault();
  int clear_galvo_sensor_fault();
  int set_galvo_mirror_fault();
  int clear_galvo_mirror_fault();
  int set_max_distance_fault();
  int clear_max_distance_fault();

  void update_ring_id_table();
  RingIdConverterInterface *get_ring_id_converter() override;

 protected:
  void add_config(Config *c);
  void remove_config(Config *c);

 private:
  void init_();
  InnoLidarBase::State get_state_();
  bool is_live_lidar_() const;

  void *alloc_buffer_(size_t size);
  void free_buffer_(void *buffer);
  void add_deliver_job_(void *);

 private:
  /* source info */
  enum LidarSource lidar_source_;
  char *ip_;
  uint16_t port_;
  bool use_tcp_;
  uint16_t udp_port_;

  char *filename_;

  /* communication with lidar */
  LidarClientCommunication *comm_;

  MemPool *packet_pool_;
  ConsumerProducer *cp_read_;
  ConsumerProducer *cp_deliver_;

  StageClientRead *stage_read_;
  StageClientDeliver *stage_deliver_;
  bool force_xyz_pointcloud_;

  ClientStats *client_stats_;

  // save raw4 data
  RawReceiver *raw_recorder_;
  InnoThread *it_raw_recorder_;
  std::string raw_recoder_save_path_;
  uint64_t faults_save_raw_{0};

  // for time sync packet, client should buf ip port info
  // and set to server if server restart
  std::string time_sync_set_value{""};

  // The ptr of galvo mirror check
  InnoGalvoMirrorCheck *galvo_mirror_check_;
  // The ptr of max distance check
  InnoMaxDistanceCheck *max_distance_check_;

  // use ring id instead of <scan_line, ch> to identify each line
  RingIdConverter *ring_id_converter_{nullptr};
  std::mutex ring_id_set_mutex_;
};

}  // namespace innovusion
#endif  // SDK_CLIENT_LIDAR_CLIENT_H_
