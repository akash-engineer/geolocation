/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_COMMON_LIDAR_BASE_H_
#define SDK_COMMON_LIDAR_BASE_H_

#include <map>
#include <mutex>  // NOLINT
#include <string>

#include <limits>

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/resource_stats.h"
#include "sdk_common/ring_id_converter_interface.h"
#include "utils/config.h"
#include "utils/log.h"
#include "utils/utils.h"

namespace innovusion {
class InnoLidarBase {
 public:
    enum State {
    STATE_INIT = 0,
    STATE_READING,
    STATE_STOPPING,
    STATE_STOPPED,
  };
  static const uint8_t kDefaultFrameRate = 10;

 public:
  InnoLidarBase(const char *config_name,
                const char *name)
      : config_manage_(config_name) {
    name_ = strdup(name);
    handle_ = 0;
    get_host_time_ = NULL;
    message_callback_external_ = NULL;
    data_packet_callback_ = NULL;
    status_packet_callback_ = NULL;
    get_host_time_ = get_host_time_default;

    for (uint32_t i = 0; i < INNO_RECORDER_CALLBACK_TYPE_MAX; i++) {
      recorder_callbacks_[i] = NULL;
      recorder_callbacks_context_[i] = NULL;
    }
    cpusetsize_ = 0;
    cpuset_ = NULL;
    exclude_callback_thread_ = 0;

    last_stage_is_up_ = false;
    callback_context_ = NULL;
  }

  virtual ~InnoLidarBase() {
    if (name_) {
      free(name_);
      name_ = NULL;
    }
  }

 public:  // static methods
  static int add_lidar(InnoLidarBase *l);
  static int remove_lidar(int handle);
  static int remove_lidar_all();
  static int stop_lidar_all();
  static InnoLidarBase *find_lidar(int handle);
  static double get_host_time_default(void *);

 public:
  inline double get_current_host_time() {
    return get_host_time_(callback_context_);
  }

  inline double get_monotonic_raw_time() {
    return InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW) / 1000000000.0;
  }
  inline static double get_monotonic_raw_time_us() {
    return InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW) / 1000.0;
  }
  inline static double get_monotonic_raw_time_ms() {
    return InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW) / 1000000.0;
  }
  inline uint32_t get_monotonic_raw_time_sec() {
    return InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW) / 1000000000;
  }

 public:
  virtual int read_ps_reg(uint16_t off, uint32_t *value) = 0;
  virtual int read_pl_reg(uint16_t off, uint32_t *value) = 0;
  virtual int write_ps_reg(uint16_t off, uint32_t value) = 0;
  virtual int write_pl_reg(uint16_t off, uint32_t value) = 0;
  virtual int set_params_file(const char *lidar_model,
                              const char *yaml_filename) = 0;
  virtual int set_config_name_value(const char *name,
                                    const char *value) = 0;
  virtual int set_reflectance_mode(enum InnoReflectanceMode) = 0;
  virtual int set_return_mode(InnoMultipleReturnMode ret_mode) = 0;
  virtual int set_roi(double hori_angle, double v_angle) = 0;
  virtual int get_roi(double *h_angle, double *v_angle) = 0;
  virtual int set_mode(enum InnoLidarMode mode,
                       enum InnoLidarMode *mode_before_change,
                       enum InnoLidarStatus *status_before_change) = 0;
  virtual int get_mode_status(enum InnoLidarMode *mode,
                              enum InnoLidarMode *pre_mode,
                              enum InnoLidarStatus *status,
                              uint64_t *in_transition_mode_ms) = 0;
  virtual int get_attribute(const char *attribute,
                            double *value) = 0;
  virtual int get_attribute_string(const char *attribute,
                                   char *buf, size_t buf_size) = 0;
  virtual int set_attribute_string(const char *attribute,
                                   const char *buf) = 0;
  virtual int set_faults_save_raw(uint64_t value) = 0;
  virtual int set_motion_compensation(double velocity[3],
                                      double angular_velocity[3]) = 0;
  virtual int thread_setaffinity_np(size_t cpusetsize,
                                    const cpu_set_t *cpuset,
                                    int exclude_callback_thread) = 0;
  virtual int get_fw_state(InnoLidarState *state, int *error_code) = 0;
  virtual int get_fw_version(char *buffer, int buffer_len) = 0;
  virtual int get_sn(char *buffer, int buffer_len) = 0;
  virtual int get_model(char *buffer, int buffer_len) = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void print_stats() = 0;
  virtual RingIdConverterInterface *get_ring_id_converter() = 0;
  virtual const char *get_name(void) const {
    return name_;
  }

 public:
  void set_callbacks(InnoMessageCallback message_callback,
                     InnoDataPacketCallback data_callback,
                     InnoStatusPacketCallback status_callback,
                     InnoHosttimeCallback get_host_time,
                     void *callback_context) {
    message_callback_external_ = message_callback;
    data_packet_callback_ = data_callback;
    status_packet_callback_ = status_callback;
    if (get_host_time) {
      get_host_time_ = get_host_time;
    }
    callback_context_ = callback_context;
    return;
  }

  int set_recorder_callback(enum InnoRecorderCallbackType type,
                            InnoRecorderCallback callback,
                            void *ctx) {
    inno_log_panic_if_not(type < INNO_RECORDER_CALLBACK_TYPE_MAX &&
                          type > INNO_RECORDER_CALLBACK_TYPE_NONE,
                          "invalid data callback type %d",
                          type);

    std::unique_lock<std::mutex> lk(recorder_callbacks_mutex_[type]);
    if (recorder_callbacks_[type]) {
      // already has callback
      return -1;
    }
    recorder_callbacks_[type] = callback;
    recorder_callbacks_context_[type] = ctx;
    return 0;
  }

  /**
   * [CST Bugfinder Defect ID 53728] Reviewed
   *
   * PolySpace report a defect here:
   * Unnecessary code, if-condition is always true.
   *
   * Avoid multithreaded competition consumption ,
   *
   * So ignore this defect
   */
  inline bool has_recorder_callback(enum InnoRecorderCallbackType type) {
    if (recorder_callbacks_[type]) {
      std::unique_lock<std::mutex> lk(recorder_callbacks_mutex_[type]);
      if (recorder_callbacks_[type]) {
        return true;
      }
    }
    return false;
  }

  /**
   * [CST Bugfinder Defect ID 53727] Reviewed
   *
   * PolySpace report a defect here:
   * Unnecessary code, if-condition is always true.
   *
   * Avoid multi-threaded competition consumption ,
   *
   * So ignore this defect
   */
  inline int do_recorder_callback(enum InnoRecorderCallbackType type,
                                   const char *buffer, int len) {
    if (recorder_callbacks_[type]) {
      InnoRecorderCallback callback = NULL;
      {
        std::unique_lock<std::mutex> lk(recorder_callbacks_mutex_[type]);
        callback = recorder_callbacks_[type];
      }
      if (callback) {
        // do not make the callback while hold the lock
        int ret = callback(handle_,
                           recorder_callbacks_context_[type],
                           type, buffer, len);
        if (ret != 0) {
          inno_log_info("recorder_callback[%d] return %d, cancel callback",
                        type, ret);
          // this is the only place the reset the callbck
          std::unique_lock<std::mutex> lk(recorder_callbacks_mutex_[type]);
          recorder_callbacks_[type] = NULL;
          recorder_callbacks_context_[type] = NULL;
        }
        return ret;
      }
    }
    return 0;
  }

  void do_message_callback_fmt(enum InnoMessageLevel error_level,
                               enum InnoMessageCode code,
                               const char *fmt,
                               ...) {
    va_list valist;
    va_start(valist, fmt);
    do_message_callback_v_(error_level, code, fmt, valist);
    va_end(valist);
  }

  int do_data_callback(const InnoDataPacket *data) {
    if (data_packet_callback_) {
      ++stat_point_data_packet_sent_;
      stat_point_sent_ += data->item_number;
      return data_packet_callback_(handle_, callback_context_, data);
    } else {
      return 0;
    }
  }

  void do_message_callback(enum InnoMessageLevel error_level,
                           enum InnoMessageCode code,
                           const char *error_message) {
    if (code != INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT) {
      inno_log_with_level(InnoLogLevel(error_level),
                        "message_callback: name=%s level=%d, "
                        "code=%d, message=%s",
                        name_, error_level, code, error_message);
    }
    if (message_callback_external_) {
      ++stat_message_packet_sent_;
      message_callback_external_(handle_, callback_context_,
                                 0,  // local message
                                 error_level, code, error_message);
    }
  }

  void do_status_callback(const InnoStatusPacket *pkt) {
    if (status_packet_callback_) {
      status_packet_callback_(handle_,
                              callback_context_,
                              pkt);
      stats_update_packet_bytes(ResourceStats::PACKET_TYPE_STATUS,
                                1, sizeof(InnoStatusPacket));
    }
  }

  virtual void stats_update_packet_bytes(
      enum ResourceStats::PacketType type,
      size_t packet, size_t byte) = 0;

  virtual void set_save_raw_data_flag(std::string cause) {
    // do nothing in LidarBase
    return;
  }

 protected:
  virtual State get_state_() = 0;

  void do_message_callback_v_(enum InnoMessageLevel error_level,
                              enum InnoMessageCode code,
                              const char *fmt,
                              va_list valist) {
    static const size_t kMaxMessageSize = 1024;
    char buffer[kMaxMessageSize];
    buffer[0] = 0;
    vsnprintf(buffer, kMaxMessageSize, fmt, valist);
    buffer[kMaxMessageSize - 1] = 0;
    do_message_callback(error_level, code, buffer);
  }

  void set_play_rate_(int rate) {
    inno_log_info("Setting play rate to %d", rate);
    if (rate > 100) {
      play_rate_ = 0;
      play_rate_x_ = rate / 10000.0;
      inno_log_info("Setting play rate to %fX",
                    play_rate_x_);
    } else if (rate >= 0) {
      play_rate_ = rate;
      play_rate_x_ = 0;
      inno_log_info("Setting play rate to %d MB/s",
                    play_rate_);
    } else {
      inno_log_verify(false, "Invalid play rate entered %d", rate);
    }
  }

 public:
  static std::mutex static_mutex_s;
  static std::map<int, InnoLidarBase*> lidars_s;
  static int max_handle_s;
  static uint32_t open_count_s;

 protected:
  int handle_;
  /* lidar ID */
  char *name_;

  int play_rate_;
  double play_rate_x_;

  /* callbacks*/
  InnoMessageCallback message_callback_external_;
  InnoDataPacketCallback data_packet_callback_;
  InnoStatusPacketCallback status_packet_callback_;
  InnoHosttimeCallback get_host_time_;
  InnoRecorderCallback recorder_callbacks_[INNO_RECORDER_CALLBACK_TYPE_MAX];
  void *recorder_callbacks_context_[INNO_RECORDER_CALLBACK_TYPE_MAX];
  std::mutex recorder_callbacks_mutex_[INNO_RECORDER_CALLBACK_TYPE_MAX];
  void *callback_context_;

  /* cpuset */
  size_t cpusetsize_;
  cpu_set_t *cpuset_;
  bool exclude_callback_thread_;

  /* lidar basic configs */
  ConfigManager config_manage_;

  std::mutex last_stage_mutex_;
  bool last_stage_is_up_;

  /* stats */
  uint64_t stat_point_sent_{0};
  uint64_t stat_point_data_packet_sent_{0};
  uint64_t stat_message_packet_sent_{0};
};

}  // namespace innovusion
#endif  // SDK_COMMON_LIDAR_BASE_H_
