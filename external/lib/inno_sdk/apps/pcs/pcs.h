  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_PCS_H_
#define PCS_PCS_H_

#include <stdint.h>
#include <time.h>

#include <condition_variable>  // NOLINT
#include <mutex>               // NOLINT
#include <string>

#include "src/sdk_common/inno_lidar_api.h"
#include "src/utils/inno_lidar_log.h"
#include "src/utils/inno_thread.h"
#include "pcs/command_test.h"
#include "pcs/dtc_manager.h"
#include "pcs/udp_listener.h"

namespace innovusion {

class CframeConverter;
class CommandParser;
class CommandTest;
class DataRecorder;
class DtcManager;
class InnoPcFrameCapture;
class InnoPcNpyRecorder;
class LidarSource;
class PcServerWsProcessor;
class PngRecorder;
class RosbagRecorder;
class UdpLogListener;
class UdpSender;
class TimeSyncUdpListener;

class PCS {
 public:
  explicit PCS(const CommandParser &c);
  ~PCS();

 public:
  static void write_log_s(PCS* pcs,
               int level, int code,
               const char *header1,
               const char *header2,
               const char *message,
               bool is_log);

 private:
  static void log_callback_s_(void *ctx, enum InnoLogLevel level,
                              const char *header1, const char *header2,
                              const char *msg) {
    (reinterpret_cast<PCS *>(ctx))->log_callback_(level, header1,
                                                  header2, msg);
  }

  static int recorder_callback_s_(int lidar_handle, void *ctx,
                                   enum InnoRecorderCallbackType type,
                                   const char *buffer, int len) {
    return (reinterpret_cast<PCS *>(ctx))->recorder_callback_(type,
                                                              buffer, len);
  }

  static void message_callback_s_(int lidar_handle, void *ctx,
                                  uint32_t from_remote,
                                  enum InnoMessageLevel level,
                                  enum InnoMessageCode code,
                                  const char *error_message) {
    (reinterpret_cast<PCS *>(ctx))->message_callback_(from_remote, level, code,
                                                      error_message);
  }

  static int data_callback_s_(int lidar_handle, void *ctx,
                              const InnoDataPacket *pkt) {
    return (reinterpret_cast<PCS *>(ctx))->data_callback_(pkt);
  }

  static int status_callback_s_(int lidar_handle, void *ctx,
                                const InnoStatusPacket *pkt) {
    return (reinterpret_cast<PCS *>(ctx))->status_callback_(pkt);
  }

 private:
  bool has_ws_(void) const {
    return ws_ != NULL;
  }
  bool is_shutdown_(void) const {
    return shutdown_;
  }

  bool checke_udp_port_conflict_(uint16_t raw_port);

  void setup_udp_(const std::string &udp_ip, uint16_t port,
                  std::mutex *mutex, UdpSender **sender);
  void setup_udps_(const std::string &udp_ip, uint16_t data_port,
                   uint16_t message_port, uint16_t status_port);
  void setup_raw_udps_(const std::string &udp_ip, uint16_t raw_port);
  int setup_time_sync_listener_(const std::string &name,
                                const std::string &ip_in,
                                uint16_t port_in,
                                const std::string &ip_out,
                                uint16_t port_out);
  void write_log_(int level, int code, const char *header1, const char *header2,
                  const char *message, bool is_log);
  void log_callback_(enum InnoLogLevel level, const char *header1,
                     const char *header2, const char *msg);
  int recorder_callback_(enum InnoRecorderCallbackType type,
                         const char *buffer, int len);
  void message_callback_(uint32_t from_remote, enum InnoMessageLevel level,
                         enum InnoMessageCode code, const char *msg);
  int data_callback_(const InnoDataPacket *pkt);
  int status_callback_(const InnoStatusPacket *pkt);
  bool is_shutdown_();
  void wait_until_shutdown_();
  void set_up_long_duration_test_mode_();
  void clear_long_duration_test_mode_();

  int raw4_send_(const InnoRaw4Packet *raw_packet);

 public:
  int get_pcs(const std::string &name,
              const std::string &value,
              std::string *result,
              void *conn, bool external);
  int set_pcs(const std::string &name, const std::string &value, void *conn);
  int add_capture_job(const std::string &type, const std::string &duration,
                      void *conn);
  const char *get_sn() const;
  int set_recorder_callback(enum InnoRecorderCallbackType type,
                            InnoRecorderCallback callback,
                            void *ctx);
  InnoThread *get_command_test_thread() const;
  PcServerWsProcessor *get_ws() const;
  int set_lidar_attribute(const std::string &name, const std::string &value);
  void pause_lidar();
  void resume_lidar();
  void stop();
  void run();

  const char *get_lidar_ip() const;
  uint32_t get_lidar_port() const;
  uint16_t get_time_sync_type() const;

  /**
   * send file(s), you can specify which file(s) to be send by <type> or
   * <path,offset,length>
   * @param item candidate: "log_snapshot" ...
   * @param path file path, if type was specified, will ignore this parameter
   * @param offset for path
   * @param length for path, in byte, <0 means no limit
   * @return 0 success
   *         400 invalid parameter
   *         50x internal error
   */
  int send_file(const std::string &item,
                const std::string &path,
                const std::string &offset,
                const std::string &length,
                void *conn);

 private:
  int load_config_line_(const char *l);
  void load_config_(const char *filename);
  int log_snapshot_(const std::string &value);
  void log_snapshot_do_(std::string tmp_file,
                               std::string path);

 private:
  static const uint32_t kErrorLogSize = 100 * 1000;
  // smaller than max packet size
  static const uint32_t kMaxStartingLogSize = 64 * 1000;
  static const uint32_t kStartingLogMaxLines = 700;
  static const uint32_t kLongDurationLogSaveIntervalSec = 5;

 private:
  const CommandParser &cmd_parser_;
  struct timespec start_timespec_;

  std::string effective_udp_client_ip_;
  uint16_t effective_data_port_;
  uint16_t effective_message_port_;
  uint16_t effective_status_port_;
  uint16_t effective_raw_port_;

  UdpSender *data_udp_sender_;
  UdpSender *status_udp_sender_;
  UdpSender *message_udp_sender_;
  UdpSender *status_local_udp_sender_;
  std::mutex data_udp_mutex_;
  std::mutex status_udp_mutex_;
  std::mutex message_udp_mutex_;
  std::mutex status_local_udp_mutex_;
  std::mutex time_sync_udp_listener_mutex_;

  // raw4
  std::string effective_udp_raw_ip_;
  UdpSender *raw_udp_sender_;
  std::mutex raw_udp_mutex_;
  uint32_t raw_udp_field_idx_;

  //
  CframeConverter *cframe_converter_;
  DataRecorder *data_recorder_;
  DataRecorder *bad_data_recorder_;
  InnoPcNpyRecorder *inno_pc_npy_recorder_;
  RosbagRecorder *rosbag_recorder_;
  PngRecorder *png_recorder_;
  InnoPcFrameCapture *frame_capturer_;
  PcServerWsProcessor *ws_;
  LidarSource *lidar_;
  UdpLogListener *fw_log_listener_;
  TimeSyncUdpListener *time_sync_udp_listener_;
  DtcManager* dtc_manager_;
  CommandTest *command_test_;
  InnoThread *it_command_test_;
  const char *long_duration_test_log_;
  bool in_long_duration_test_mode_;
  uint64_t long_duration_log_last_save_time_;

  size_t cframe_received_;

  // last status
  InnoStatusPacket last_pkt;

  bool shutdown_;
  size_t message_id_;
  size_t message_log_id_;
  size_t status_id_;
  InnoStatusPacket status_packet_;
  InnoStatusPacket last_status_packet_;

  uint32_t error_log_offset_;
  uint32_t error_log_warning_lines_;
  uint32_t error_log_error_lines_;
  char error_log_[kErrorLogSize];
  bool assert_failure_called_;
  bool faults_update_freq_control_;

  uint32_t starting_log_lines_;
  uint32_t starting_log_offset_;
  char starting_log_[kMaxStartingLogSize];

  std::mutex mutex_;
  std::recursive_mutex recur_mutex_;
  std::condition_variable cond_;

  uint16_t last_time_sync_type_;
  // log snapshot
  bool is_doing_snapshot_{false};
  std::mutex log_snapshot_mutex_;
  enum LogSnapshotStatus {
    LOG_SNAPSHOT_SUCCESS = 0,
    LOG_SNAPSHOT_NONE = 1,
    LOG_SNAPSHOT_DOING = 2,
    LOG_SNAPSHOT_FAILED = 3,
  } log_snapshot_status_{LOG_SNAPSHOT_NONE};
};

}  // namespace innovusion

#endif  // PCS_PCS_H_
