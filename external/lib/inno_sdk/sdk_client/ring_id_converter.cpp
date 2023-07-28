/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <vector>
#include <sstream>
#include <cfloat>
#include <algorithm>
#include "sdk_client/ring_id_converter.h"
#include "sdk_client/lidar_client.h"
#include "../sdk_common/inno_lidar_packet.h"
#include "../utils/inno_lidar_log.h"
#include "../utils/utils.h"

namespace innovusion {

RingIdConverter::RingIdConverter(InnoLidarClient *lidar) {
  lidar_ = lidar;
  init_success_ = (setup_ring_id_table_normal_() == 0 &&
                   setup_ring_id_table_quiet_() == 0);
}

RingIdConverter::~RingIdConverter() {
}

int RingIdConverter::update_ring_id_table() {
  // get_galvo_first_period
  char buffer[20] = "\0";
  int ret = lidar_->get_attribute_string("galvo_first_period",
                                         buffer, sizeof(buffer));
  if (ret) {
    inno_log_error("get galov_first_period failed, stop update ring id table");
    return ret;
  }
  galvo_periods_[0] = std::stoi(buffer);
  // get first roi scanline idx both for up-frame and down-frame
  // N_UP = int[(galvo_first_period / 15000) / (12 / mtr_f_rpm)]
  // N_DOWN = int[(1/galvo_framerate -
  //  (galvo_first_period+galvo_second_period
  //   + galvo_third_period
  //   + galvo_fourth_period)/15000) / (12 / mtr_f_rpm)]
  uint16_t first_roi_line_up
      = floor((static_cast<double>(galvo_periods_[0])
          / InnoConsts::kGalvoPeridUnitsPerSecond)
                  / single_line_scan_time_s_);
  uint16_t first_roi_line_down = floor((1.0 / frame_rate_ -
      static_cast<double >(galvo_periods_[0] + galvo_periods_[1]
          + galvo_periods_[2] + galvo_periods_[3])
          / InnoConsts::kGalvoPeridUnitsPerSecond) / single_line_scan_time_s_);
  if (first_roi_line_up == current_first_roi_line_up_ &&
      first_roi_line_down == current_first_roi_line_down_) {
    inno_log_trace("roi didn't change, stop update ring id table");
    return 0;
  }
  current_first_roi_line_up_ = first_roi_line_up;
  current_first_roi_line_down_ = first_roi_line_down;
  inno_log_trace("line_num_per_frame=%d, "
                 "single_line_scan_time_s_ = %lf, "
                 "1st_roi_up=%u, 1st_roi_down=%u, "
                 "galvo_periods=%d/%d/%d/%d",
                 line_num_per_frame_,
                 single_line_scan_time_s_,
                 first_roi_line_up, first_roi_line_down,
                 galvo_periods_[0], galvo_periods_[1],
                 galvo_periods_[2], galvo_periods_[3]);
  fill_table_(first_roi_line_up, first_roi_line_down,
              line_num_per_frame_, ring_id_table_[1]);
  return 0;
}

int RingIdConverter::setup_ring_id_table_normal_() {
  // get some params which are stable during running
  // get motor config
  char buffer[1024] = "\0";
  int ret = lidar_->get_attribute_string("internal_motor_config",
                                         buffer, sizeof(buffer));
  if (ret) {
    inno_log_error("get motor config error");
    return ret;
  }

  std::unordered_map<std::string, int32_t> map = get_config_map_(buffer);

  // galvo second/third/fourth period from internal config
  memset(galvo_periods_, 0, sizeof(galvo_periods_));
  galvo_periods_[0] = map["galvo_first_period"];
  galvo_periods_[1] = map["galvo_second_period"];
  galvo_periods_[2] = map["galvo_third_period"];
  galvo_periods_[3] = map["galvo_fourth_period"];

  // get how many scan lines per frame base on mtr_f_rpm and frame rate
  uint32_t mtr_f_rpm = 0;
  mtr_f_rpm = map["mtr_f_rpm"];
  inno_log_verify(mtr_f_rpm != 0 && mtr_f_rpm <= InnoConsts::kMaxRPM,
                  "invalid mtr_f_rpm: %d", mtr_f_rpm);
  frame_rate_ = 0;
  frame_rate_ = map["galvo_framerate"];
  inno_log_verify(frame_rate_ > 0, "invalid frame rate: %d", frame_rate_);
  single_line_scan_time_s_ =
      60.0 / mtr_f_rpm / InnoConsts::kPolygonFacet;
  line_num_per_frame_ =
      InnoConsts::kPolygonFacet * mtr_f_rpm / 60 / frame_rate_;

  // get roi num base on pattern, this should be stable during running
  if (galvo_periods_[2] == 0) {
    roi_num_ = 1;
  } else {
    roi_num_ = 2;
  }

  return update_ring_id_table();
}

int RingIdConverter::setup_ring_id_table_quiet_() {
  // get motor speed and frame_rate config of quiet mode
  char buffer[1024] = "\0";
  int ret = lidar_->get_attribute_string("internal_quiet_config",
                                         buffer, sizeof(buffer));
  if (ret) {
    inno_log_error("get motor config error");
    return ret;
  }
  std::unordered_map<std::string, int32_t> map = get_config_map_(buffer);
  uint32_t mtr_f_rpm = 0;
  mtr_f_rpm = map["mtr_f_rpm"];
  inno_log_verify(mtr_f_rpm != 0 && mtr_f_rpm <= InnoConsts::kMaxRPM,
                  "invalid mtr_f_rpm: %d", mtr_f_rpm);
  uint32_t frame_rate = 0;
  frame_rate = map["galvo_framerate"];
  inno_log_verify(frame_rate > 0, "invalid frame rate: %d", frame_rate);

  uint32_t line_num_per_frame =
      InnoConsts::kPolygonFacet * mtr_f_rpm / 60 / frame_rate;

  fill_table_(UINT16_MAX, UINT16_MAX, line_num_per_frame, ring_id_table_[0]);
  return 0;
}

void RingIdConverter::fill_table_(uint16_t first_roi_line_up,
                                  uint16_t first_roi_line_down,
                                  uint16_t line_num_per_frame,
                                  uint16_t table[INNO_FRAME_DIRECTION_MAX]\
                                            [InnoConsts::kMaxScanLinePerFrame]\
                                            [kInnoChannelNumber]) {
  // get each scan line's v_angle theoretically
  class ScanLineVirticalAngle {
   public:
    uint16_t scan_line;
    uint8_t ch;
    double v_angle;
  };
  std::vector<ScanLineVirticalAngle>
      line_angles(line_num_per_frame * kInnoChannelNumber);
  inno_log_verify(INNO_FRAME_DIRECTION_MAX <= 2, "INNO_FRAME_DIRECTION_MAX");
  // use first_roi_line_up for both two scan direction
  uint16_t first_roi_line[2] = {first_roi_line_up, first_roi_line_up};
  for (uint32_t scan_direct = 0;
       scan_direct < INNO_FRAME_DIRECTION_MAX; ++scan_direct) {
    for (uint16_t scan_line = 0, idx = 0;
         scan_line < line_num_per_frame;
         ++scan_line) {
      for (uint8_t ch = 0; ch < kInnoChannelNumber; ++ch, idx++) {
        line_angles[idx].scan_line = scan_line;
        line_angles[idx].ch = ch;
        line_angles[idx].v_angle
            = get_v_angle_of_scanline_(scan_line, ch,
                                       first_roi_line[scan_direct]);
        inno_log_trace("set line_angles[%d] to \tsc=%d\tch=%d\tv_angle=%lf",
                       idx, scan_line, ch,
                       line_angles[idx].v_angle);
      }
    }

    // sort all scan lines by v_angle
    // to get <<scan line idx, ch>, ring-id> table
    std::sort(line_angles.begin(), line_angles.end(),
              [](const ScanLineVirticalAngle &a,
                 const ScanLineVirticalAngle &b) {
                return a.v_angle < b.v_angle;
              });

    // init table, direction 0 = down frame
    uint16_t ring_id = 0;
    for (uint16_t i = 0; i < line_num_per_frame * kInnoChannelNumber; ++i) {
      if (line_angles[i].v_angle != -DBL_MAX) {
        ring_id++;
      }
      // revert scan_line in down-frame(direction = 0)
      uint16_t scan_line = scan_direct ? line_angles[i].scan_line :
                           (line_num_per_frame - line_angles[i].scan_line - 1);
      inno_log_trace("will set ring_id_table_[%d][%d][%d]=%d,"
                     "from line_angles[%d], v_angle=%lf",
                     scan_direct, scan_line, line_angles[i].ch, ring_id,
                     i, line_angles[i].v_angle);
      table[scan_direct][scan_line][line_angles[i].ch] = ring_id;
    }
  }
  // for debug
  if (inno_log_level_g >= INNO_LOG_LEVEL_TRACE) {
    for (int i = 0; i < 2; ++i) {
      std::ostringstream ss;
      ss << i << "-";
      for (uint32_t j = 0;
           j < line_num_per_frame; ++j) {
        ss << "line[" << j << "]-channel[0/1/2/3]:";
        for (int ch = 0; ch < 4; ++ch) {
          ss << " " << table[i][j][ch];
        }
        ss << std::endl;
      }
      inno_log_trace("ring_id table: %s", ss.str().c_str());
    }
  }
}

double RingIdConverter::get_v_angle_of_scanline_(uint32_t scanline,
                                                 uint32_t ch,
                                                 uint32_t first_roi_line)
                                                 const {
  uint32_t x = scanline;
  uint32_t N = first_roi_line;
  uint32_t M = roi_num_;
  if (x < N) {
    return (x+1)*0.96 + 1.2*ch;
  } else if (x < N + 12) {
    return (x-N)*0.1 + (N+1)*0.96 + 1.2*ch;
  } else if (x == N + 12) {
    return -DBL_MAX;
  } else if (x <= N + 12*M + 1) {
    return (x-N-1)*0.1 + (N+1)*0.96 + 1.2*ch + 3.6;
  } else {
    return (x-12*M)*0.96 + 1.2*M + 1.2*ch +3.6;
  }
}

std::unordered_map<std::string, int32_t>
RingIdConverter::get_config_map_(const char *buffer) const {
  std::ostringstream ss;
  ss << buffer;
  inno_log_trace("motor config: %s", ss.str().c_str());
  std::vector<std::string> items = InnoUtils::split(ss.str(), "\n");
  std::unordered_map<std::string, int32_t> map;
  for (const std::string& item : items) {
    std::vector<std::string> key_value = InnoUtils::split(item, " = ");
    if (key_value.size() == 2) {
      map.emplace(key_value[0], std::stoi(key_value[1]));
      inno_log_trace("%s:%s", key_value[0].c_str(), key_value[1].c_str());
    } else {
      inno_log_info("ignore config: %s", item.c_str());
    }
  }
  return map;
}

}  // namespace innovusion

