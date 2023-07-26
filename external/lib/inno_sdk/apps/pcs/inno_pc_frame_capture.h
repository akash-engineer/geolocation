/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_INNO_PC_FRAME_CAPTURE_H_
#define PCS_INNO_PC_FRAME_CAPTURE_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <condition_variable>  // NOLINT
#include <deque>
#include <memory>
#include <mutex>   // NOLINT
#include <string>
#include <thread>  // NOLINT

#include "src/sdk_common/inno_lidar_api.h"
#include "src/ws_utils/server_ws.h"

namespace innovusion {

class MemPool;
class PCS;
class PngRecorder;
class RosbagRecorder;

class InnoPcFrameCapture {
 private:
  enum CaptureType {
    CAPTURE_TYPE_NONE = 0,
    CAPTURE_TYPE_PCD = 1,
    CAPTURE_TYPE_NPY = 2,
    CAPTURE_TYPE_PC = 3,
    CAPTURE_TYPE_RAW = 4,
    CAPTURE_TYPE_BAG = 5,
    CAPTURE_TYPE_PNG = 6,
    CAPTURE_TYPE_PC_XYZ = 7,
    CAPTURE_TYPE_PCD_BINARY = 8,
    CAPTURE_TYPE_CSV = 9,
    CAPTURE_TYPE_RAW_RAW = 10,
    CAPTURE_TYPE_YAML = 11,
    CAPTURE_TYPE_MAX,
  };

  enum BagStreamStatus {
    BAG_STREAM_INIT = 0,
    BAG_STREAM_START = 1,
    BAG_STREAM_SENDING = 2,
    BAG_STREAM_END = 3,
    BAG_STREAM_ERROR = 4,
  };

  DEFINE_INNO_COMPACT_STRUCT(PcdPoint) {
    float x;
    float y;
    float z;
    uint16_t intensity;
    uint8_t channel;
    uint8_t roi;
    uint8_t facet;
    uint8_t is_2nd_return;
    uint8_t multi_return;
    uint8_t confid_level;
    uint8_t flags;
    uint8_t elongation;
    double timestamp;
    uint16_t scanline;
    uint16_t scan_idx;
    uint32_t frame_id;
  };

 private:
  static const size_t kMaxBlockNum = 1000 * 1000;
  static const size_t kMaxPacketNum = 1000 * 100;
  static const size_t kMaxPacketSize = 65536;
  static const size_t kPacketPoolSize = 500;

 private:
  static int recorder_callback_raw2_s_(int lidar_handle, void *ctx,
                                       enum InnoRecorderCallbackType type,
                                       const char *buffer, int len) {
    return (reinterpret_cast<InnoPcFrameCapture *>(ctx))->
        recorder_callback_raw2_(type, buffer, len);
  }
  static int write_to_wconn_s_(void *ctx, const void *buf, size_t buf_len);

 public:
  explicit InnoPcFrameCapture(PCS *pcs);
  ~InnoPcFrameCapture();

 public:
  bool has_active_capture_job() {
    std::unique_lock<std::mutex> lk(mutex_);
    return job_type_ > CAPTURE_TYPE_NONE &&
        job_type_ < CAPTURE_TYPE_MAX;
  }

  int add_capture_job(const std::string &type,
                      const std::string &duration,
                      void *ctx);
  void received_data_packet(const InnoDataPacket *pkt);
  void stop_capture_force() {
    std::unique_lock<std::mutex> lk(mutex_);
    if (job_type_ != CAPTURE_TYPE_RAW_RAW &&
        job_type_ != CAPTURE_TYPE_RAW) {
      inno_log_info("currently only support stop capture raw/raw_raw");
      return;
    }
    stop_force_ = true;
  }

 private:
  void reset_job_state_with_lock_();
  void pcd_received_data_packet_(const InnoDataPacket *pkt);
  void pc_received_data_packet_(const InnoDataPacket *pkt);
  void png_received_data_packet_(const InnoDataPacket *pkt);

  void pcd_add_data_packet_(const InnoDataPacket *pkt);

  void clear_pc_packet_deque_w_lock_();
  void end_worker_with_lock_();
  void end_worker_();
  void end_job_with_lock_();
  void end_job_();

  int do_send_bag_capture_(InnoDataPacket *pkt);

  void send_raw_raw_capture_thread_();
  void send_bag_capture_thread_();
  void send_png_capture_thread_();
  void send_pcd_capture_thread_();
  void send_inno_pc_capture_thread_();

  int recorder_callback_raw2_(enum InnoRecorderCallbackType type,
                              const char *buffer, int len);
  void prepare_http_header(char *buffer, size_t buffer_len,
                           char duration_type, ssize_t duration,
                           const char *type, const char *ext);
  int write_to_wconn_(const void *buf, size_t buf_len);

 private:
  static const char *pcd_header_;
  static const char *pcd_row_;
  static const char *csv_header_;
  static const char *csv_row_;

 private:
  PCS *pcs_;
  enum CaptureType job_type_;
  size_t job_duration_;
  size_t display_job_duration_;
  int raw_raw_channel_;
  std::shared_ptr<WsConnection> job_conn_;

  bool reflectance_;

  bool pcd_in_sending_;
  InnoDataPacket *pcd_saved_packet_;
  InnoDataPacket *pcd_saved_packet_headers_;
  uint32_t pcd_block_so_far_;
  uint32_t pcd_packet_so_far_;
  int64_t pcd_start_frame_;
  int64_t pcd_last_frame_;
  uint64_t pcd_frame_captured_;
  uint32_t current_point_in_frame_;

  MemPool *pc_packet_pool_;
  std::deque<InnoDataPacket *> pc_packet_deque_;
  bool pc_shutdown_;
  bool null_enqueued_;
  std::condition_variable pc_cond_;
  std::string capture_filename_;

  size_t raw2_sent_bytes_ = 0;
  size_t raw2_callback_ = 0;
  uint32_t bag_stream_status_;
  RosbagRecorder *rosbag_stream_;

  PngRecorder* png_stream_;

  std::thread *worker_;
  std::mutex mutex_;

  bool stop_force_{false};
};

}  // namespace innovusion

#endif  // PCS_INNO_PC_FRAME_CAPTURE_H_
