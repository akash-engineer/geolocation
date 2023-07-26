/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_RING_ID_CONVERTER_H_
#define SDK_CLIENT_RING_ID_CONVERTER_H_

#include <mutex>  // NOLINT
#include <string>
#include <unordered_map>
#include "utils/types_consts.h"
#include "utils/utils.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/ring_id_converter_interface.h"

namespace innovusion {
class InnoLidarClient;
class RingIdConverter: public RingIdConverterInterface {
 public:
  explicit RingIdConverter(InnoLidarClient *lidar);
  ~RingIdConverter();
  int update_ring_id_table();
  bool valid() {
    return init_success_;
  }

  inline uint16_t get_ring_id(InnoLidarMode mode,
                              uint32_t scan_direction,
                              uint32_t scan_id, uint32_t ch) override {
    switch (mode) {
      case INNO_LIDAR_MODE_WORK_QUIET:
        return ring_id_table_[0][scan_direction][scan_id][ch];
      case INNO_LIDAR_MODE_WORK_NORMAL:
        return ring_id_table_[1][scan_direction][scan_id][ch];
      default:
        return 0;
    }
  }

 private:
  int setup_ring_id_table_normal_();
  int setup_ring_id_table_quiet_();
  double get_v_angle_of_scanline_(uint32_t scanline,
                                  uint32_t ch,
                                  uint32_t first_roi_line) const;
  void fill_table_(uint16_t first_roi_line_up,
                   uint16_t first_roi_line_down,
                   uint16_t line_num_per_frame,
                   uint16_t table[INNO_FRAME_DIRECTION_MAX]\
                                  [InnoConsts::kMaxScanLinePerFrame]\
                                  [kInnoChannelNumber]);
  std::unordered_map<std::string, int32_t> get_config_map_(const char *) const;

 private:
  InnoLidarClient *lidar_;
  uint8_t roi_num_{2};  // get from galvo scan pattern
  uint8_t frame_rate_{0};
  double single_line_scan_time_s_{};  // get from polygon rpm
  uint16_t line_num_per_frame_{};  // get from polygon rpm and frame rate
  uint16_t galvo_periods_[4];

  uint16_t current_first_roi_line_up_{0};
  uint16_t current_first_roi_line_down_{0};

  bool init_success_;

  // 2 for up and down scan direction
  uint16_t ring_id_table_[2]
                         [INNO_FRAME_DIRECTION_MAX]\
                         [InnoConsts::kMaxScanLinePerFrame]\
                         [kInnoChannelNumber];
};
}  // namespace innovusion
#endif  // SDK_CLIENT_RING_ID_CONVERTER_H_
