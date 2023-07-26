/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_LIDAR_FAULT_CHECK_H_
#define SDK_CLIENT_LIDAR_FAULT_CHECK_H_

#include <algorithm>
#include <vector>

#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"
#include "utils/utils.h"

namespace innovusion {

#define INNO_EPSLION_VALUE 1e-8

enum InnoFrameCheckProcess {
  INNO_FRAME_CHECK_CONTINUE = 0,
  INNO_FRAME_CHECK_LAST_COMPELETED = 1,
};

enum InnoGalvoFaultCheckStatus {
  INNO_GALVO_FAULT_STATUS_NORMAL = 0,
  INNO_GALVO_FAULT_STATUS_SET_GALVO_MIRROR = 1,
  INNO_GALVO_FAULT_STATUS_SET_GALVO_SENSOR = 2,
  INNO_GALVO_FAULT_STATUS_INVALID_CHECK_SKIP = 3,
  INNO_GALVO_FAULT_STATUS_VALID_ANGLE_R2_TOO_BIG = 4,
  INNO_GALVO_FAULT_STATUS_VALID_COUNT_NOT_ENOUGH = 5,
  INNO_GALVO_FAULT_STATUS_CHECKING = 6,
  INNO_GALVO_FAULT_STATUS_NO_EXT_REF = 7,
  INNO_GALVO_FAULT_STATUS_SPEED_NOT_ENOUGH = 8,
  INNO_GALVO_FAULT_STATUS_MAX
};

enum InnoGalvoFrameCheckCode {
  INNO_GALVO_CHECK_FRAME_CODE_NORMAL = 0,
  INNO_GALVO_CHECK_FRAME_CODE_DEVIATED = 1,
  INNO_GALVO_CHECK_FRAME_CODE_INVALID_SPEED = 2,
  INNO_GALVO_CHECK_FRAME_CODE_INVALID_EXT_REF = 3,
  INNO_GALVO_CHECK_FRAME_CODE_FIT_COUNT_NOT_ENOUGH = 4,
  INNO_GALVO_CHECK_FRAME_CODE_FIT_INVALID = 5,
  INNO_GALVO_CHECK_FRAME_CODE_INVALID_ANGLE = 6,
  INNO_GALVO_CHECK_FRAME_CODE_INVALID = 7,
  INNO_GALVO_CHECK_FRAME_CODE_EXT_TIMEOUT = 8,
  INNO_GALVO_CHECK_FRAME_CODE_SPEED_CHANGE_QUICKLY = 9,
  INNO_GALVO_CHECK_FRAME_CODE_CHECK_COUNT_NOT_ENOUGH = 10,
  INNO_GALVO_CHECK_FRAME_CODE_LAST_FRAME_IDX_OUT = 11,
  INNO_GALVO_CHECK_FRAME_CODE_INVALID_STEERING_ANGLE = 12,
  INNO_GALVO_CHECK_FRAME_CODE_MAX
};

enum InnoMaxDistanceFaultCheckStatus {
  INNO_MAX_DISTANCE_STATUS_NORMAL = 0,
  INNO_MAX_DISTANCE_STATUS_SET_FAULT = 1,
  INNO_MAX_DISTANCE_STATUS_INVALID_SPEED = 2,
  INNO_MAX_DISTANCE_STATUS_SPEED_TIMEOUT = 3,
  INNO_MAX_DISTANCE_STATUS_INVALID_COUNT = 4,
  INNO_MAX_DISTANCE_STATUS_CHECKING = 5,
  INNO_MAX_DISTANCE_STATUS_MID = 6,
  INNO_MAX_DISTANCE_STATUS_INVALID = 7,
  INNO_MAX_DISTANCE_STATUS_SPEED_TOO_LOW = 8,
  INNO_MAX_DISTANCE_STATUS_ON_TUNNEL = 9,
  INNO_MAX_DISTANCE_STATUS_MAX
};

DEFINE_INNO_COMPACT_STRUCT(InnoPointXYZT) {
  double x;
  double y;
  double z;
  uint16_t reserved;  // galvo check for timestamp 10us
                      // max distance check for refl
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoVector3D) {
  double x;
  double y;
  double z;
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoGroundCoeff) {
  double a;
  double b;
  double c;
  double d;
};
DEFINE_INNO_COMPACT_STRUCT_END

/*
 * Galvo mirror check result
 */
DEFINE_INNO_COMPACT_STRUCT(InnoGalvoCheckResult) {
  uint64_t frame;                /* current frame idx */
  double speed;                  /* current vehicle speed */
  double steering_wheel_angle;   /* the angle of steering wheel */
  uint64_t speed_update_ts_ms;   /* speed update time */
  uint64_t steering_update_ts_ms; /* steering angle update time */
  double speed_acc;              /* speed acc */
  InnoVector3D input_vector;     /* external input rotation vector */
  InnoVector3D input_delta;      /* external input delta xyz */
  InnoGroundCoeff ground_coeff;  /* coe of fitting plane */
  InnoVector3D rotated_vector;   /* new vector after rotated */
  double variance;               /* R2 of fitting plane */
  double cos_angle;              /* value of Cos(included angle) */
  double deviated_angle;         /* deviated angle */
  double mean_deviated_angle;    /* average valid angles in 50 times */
  double deviated_angle_r2;      /* R2 of deviated ange in 50 times */
  uint32_t points_count;         /* the count of points maybe on ground */
  uint8_t fault_times;           /* the count of fault occurs in 50 times */
  uint8_t valid_times;           /* valid angle times */
  /* the check result of current frame */
  InnoGalvoFrameCheckCode frame_check_code: 8;
  /* the fault status of 50 times */
  InnoGalvoFaultCheckStatus fault_status: 8;
  uint64_t use_time_us;          /* galvo check use time (us) */
};
DEFINE_INNO_COMPACT_STRUCT_END

/*
 * Max distance check result
 */
DEFINE_INNO_COMPACT_STRUCT(InnoMaxDistanceResult) {
  uint64_t latest_check_idx;     /* latest check idx */
  uint64_t latest_frame_idx;     /* latest frame idx */
  double mean_speed;             /* mean speed */
  double mean_refl;              /* mean refl */
  uint32_t valid_frame_count;    /* valid check count */
  double total_speed;
  double total_refl;
  uint32_t threshold_valid_count;
  double threshold_fault_refl;
  double threshold_normal_refl;
  double threshold_min_speed;
  double threshold_check_speed;
  InnoMaxDistanceFaultCheckStatus fault_status;
};
DEFINE_INNO_COMPACT_STRUCT_END

typedef std::vector<InnoPointXYZT> InnoCheckPoints;

/* Scan direction */
static const int kInnoScanDirectionTop2Bottom = 0;
static const int kInnoScanDirectionBottom2Top = 1;

class InnoGalvoMirrorCheck {
 public:
  static const size_t kMaxXyzDataPacketBufSize = 1024 * 1024;
  static const uint32_t kInnoGroundValidMaxCount = 2000;
  static const uint32_t kInnoGroundPointsReservedSize = 3000;
  static const int16_t kInnoGroundMatrixSize = 9;  // 3 * 3
  /* The plane normal vector of vehicle */
  static constexpr InnoVector3D vehicle_plane_vector = {0, 0, 1};
  /* Min count of ground points */
  static const uint32_t kInnoGroundValidMinCount = 100;
  static const uint64_t kInnoGalvoCheckMaxTimes = 25;
  static const int kInnoGalvoCheckMaxTimesIndex = 24;
  static const uint64_t kInnoSpeedCheckMaxTimes = 5;
  static constexpr double kInnoGroundDistanceThreshold1 = 0.1;
  static constexpr double kInnoGroundDistanceThreshold2 = 0.03;
  /* Min galvo check vehicle speed */
  static constexpr double kInnoMinGalvoVehicleSpeed = 60.0;
  /* Min speed to check */
  static constexpr double kInnoMinGalvoCheckSpeed = 30.0;
  /* Min galvo offset angle */
  /* Report INNO_LIDAR_IN_FAULT_GALVO_MIRROR */
  static constexpr double kInnoGalvoMinOffsetAngle = 2.0;
  /* Report INNO_LIDAR_IN_FAULT_GALVO_SENSOR */
  static constexpr double kInnoGalvoMinOffsetAngle2 = 3.0;
  /* Max deviated angle r2 */
  static constexpr double kInnoGalvoMaxAngleR2 = 0.05;
  static constexpr double kInnoGalvoMaxAngleR2Rest = 0.1;
  /* Min count of fault occur to report */
  static const uint8_t kInnoGalvoMinCheckTimes = 10;
  static const uint8_t kInnoGalvoMinCheckTimes2 = 20;
  /* Speed update timeout */
  static const uint64_t kInnoUpdateSpeedMaxDelayTime = 500;
  /* Speed steering wheel angle timeout */
  static const uint64_t kInnoUpdateSteeringMaxDelayTime = 500;
  /* Frame idx max diff */
  static const uint64_t kInnoFrameIdxMaxDiff = 5;
  /* Max deviated angle */
  static constexpr double kInnoMaxDeviatedAngle = 45.0;
  /* top -> bottom, check scan id range 34 - 39 */
  static const int kInnoCheckBottomScanId0 = 33;
  /* bottom -> top, check scan id range 0 - 5 */
  static const int kInnoCheckBottomMaxScanId1 = 15;
  static const int kInnoCheckBottomScanId1 = 2;
  /* Check the road within 3 - 25m in front of the vehicle */
  static constexpr double kInnoCheckMinDistance = 3.0;
  static constexpr double kInnoCheckMaxDistance = 25.0;
  /* Check the left side of the vehicle */
  static constexpr double kInnoCheckLeftEdge = -0.5;
  /* Check the right side of the vehicle */
  static constexpr double kInnoCheckRightEdge = 0.5;
  /* Divide scan idx */
  static const uint32_t kInnoGroundDivideScanIdxSize = 2;
  static constexpr double kInnoCheckValidFitPercent = 0.85;
  /* Max steering wheel angle */
  static constexpr double kInnoGalvoCheckMaxSteeringAngle = 6.0;
  static constexpr double kInnoGalvoCheckSteeringResetAngle = 12.0;

 public:
  InnoGalvoMirrorCheck();
  ~InnoGalvoMirrorCheck();

  void reset_ref_idx();
  void reset_check_times();
  void update_ext_ref(const double vec_x,
                      const double vec_y,
                      const double vec_z,
                      const double delta_x,
                      const double delta_y,
                      const double delta_z);
  void update_vehicle_speed(const double speed);
  void update_steering_wheel_angle(const double angle);
  InnoFrameCheckProcess galvo_mirror_offset_check(
                                const InnoDataPacket &pkt,
                                InnoGalvoCheckResult *check_result);

 private:
  void init_();
  bool ref_has_set_();
  int convert_rotation_vector_to_matrix_();
  int convert_rotation_angle_to_matrix_();
  int collect_ground_points_(const InnoDataPacket &pkt);
  int collect_ground_points_process_(const InnoDataPacket &pkt);
  int cal_and_get_galvo_check_result_(
                      const InnoCheckPoints &ground_points,
                      InnoGalvoCheckResult *check_result);
  double calculate_vector_module_(const InnoVector3D &vector_3d);
  double get_fitting_plane_variance_(const InnoCheckPoints &ground_points,
                                    const InnoGroundCoeff &coeff_plane);
  int fitting_plane_from_points_(const InnoCheckPoints &ground_points,
                                 InnoGroundCoeff *coeff_plane);
  double estimate_param_(const InnoPointXYZT &p0, const InnoPointXYZT &p1,
                        const InnoPointXYZT &p2, InnoGroundCoeff *plane);
  double compute_distance_(const InnoCheckPoints &in_points,
                          const InnoGroundCoeff &plane,
                          const double &module,
                          uint32_t *valid_count);
  int quick_ransac_plane_(const InnoCheckPoints &in_points,
                         InnoGroundCoeff *out_plane);
  int fitting_plane_ransac_least_(const InnoCheckPoints &in_points,
                                 const double delta_z_per_10us,
                                 InnoGroundCoeff *coeff);

 private:
  InnoVector3D input_vector_;
  InnoVector3D input_delta_;
  // Save the speed of vehicle
  double vehicle_speed_;
  // Save the angle of steering wheel
  double steering_wheel_angle_;
  // Latest update timestamp of speed
  uint64_t speed_update_ts_ms_;
  // Latest update timestamp of steering wheel angle
  uint64_t steering_update_ts_ms_;
  // The vehicle speed at frame start
  double frame_start_speed_;
  // The update timestamp of speed at frame start
  uint64_t frame_start_speed_ts_ms_;
  double frame_start_steering_angle_;
  double frame_start_steering_angle_ts_ms_;
  double last_vehicle_speed_;
  uint64_t ref_idx_;
  double rotation_matrix_[kInnoGroundMatrixSize];
  uint64_t check_times_;
  uint64_t check_speed_times_;
  double check_angles_[kInnoGalvoCheckMaxTimes];
  double check_speeds_[kInnoGalvoCheckMaxTimes];
  double total_speeds_[kInnoGalvoCheckMaxTimes];
  uint64_t last_frame_idx_;
  bool need_ground_points_;
  InnoCheckPoints check_galvo_ground_points_;
};

class InnoMaxDistanceCheck {
 public:
  /* Speed update timeout */
  static const uint64_t kInnoUpdateSpeedMaxDelayTime = 1000;
  static constexpr double kInnoTunnelRight = 15.0;
  static constexpr double kInnoTunnelLeft = -15.0;
  static constexpr double kInnoOnTunnelMinPercent = 0.98;
  static const uint32_t kInnoMinTunnelPointCount = 1000;
  static constexpr double kInnoMinHealSpeed = 20.0;
  static constexpr double kInnoMinCheckSpeed = 5.0;

 public:
  InnoMaxDistanceCheck();
  ~InnoMaxDistanceCheck();

  void restart_check();
  void update_vehicle_speed(const double speed);
  int check_data_on_tunnel(const InnoDataPacket &pkt);
  int check_max_distance(const char *buf,
                         InnoMaxDistanceResult *check_result);

 private:
  uint32_t check_idx_;
  double vehicle_speed_;
  uint64_t distance_speed_update_ts_ms_;
  uint64_t last_frame_idx_;
  uint64_t data_last_frame_idx_;
  double total_speed_;
  double total_refl_;
  uint32_t valid_check_count_;
  bool on_tunnel_;
  uint32_t on_tunnel_times_;
  uint64_t tunnel_collect_size_;
  uint64_t frame_points_size_;
};

}  // namespace innovusion
#endif  // SDK_CLIENT_LIDAR_FAULT_CHECK_H_
