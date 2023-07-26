/*
 *  Copyright (C) 2021 Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef SDK_COMMON_INNO_LIDAR_PACKET_H_
#define SDK_COMMON_INNO_LIDAR_PACKET_H_

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/************
 Innovusion LiDAR sends out pointcloud data (INNO_ITEM_TYPE_SPHERE_POINTCLOUD)
 and message data (INNO_ITEM_TYPE_MESSAGE) in InnoDataPacket format in
 UDP packets.

 In the pointcloud data, points (InnoChannelPoint) of the same firing
 cycle are organized to 1 block (InnoBlock). In each block, the horizontal
 angle and vertical angle are specifed for the channel-0. For other channels'
 angles, we need to lookup the angle-adjust-table which can be fetch from
 the LiDAR.

 Innovusion LiDAR sends out faults and detail status data in InnoStatusPacket
 format in UDP packets every 50 ms. It contains InnoStatusInFaults,
 InnoStatusExFaults, InnoStatusCounters and InnoStatusSensorReadings.
*************/

/************
 Macros
*************/
#ifndef DEFINE_INNO_COMPACT_STRUCT
#if !(defined(_MSC_VER))
#define DEFINE_INNO_COMPACT_STRUCT(x) struct __attribute__((packed)) x
#define DEFINE_INNO_COMPACT_STRUCT_END
#else
#define DEFINE_INNO_COMPACT_STRUCT(x) __pragma(pack(push, 1)) struct x
#define DEFINE_INNO_COMPACT_STRUCT_END __pragma(pack(pop))
#endif
#endif

/************
 Enums
*************/
enum InnoLidarMode {
  INNO_LIDAR_MODE_NONE = 0,
  INNO_LIDAR_MODE_SLEEP = 1,
  INNO_LIDAR_MODE_STANDBY = 2,
  INNO_LIDAR_MODE_WORK_NORMAL = 3,
  INNO_LIDAR_MODE_WORK_SHORT_RANGE = 4,
  INNO_LIDAR_MODE_WORK_CALIBRATION = 5,
  INNO_LIDAR_MODE_PROTECTION = 6,
  INNO_LIDAR_MODE_WORK_QUIET = 7,
  INNO_LIDAR_MODE_WORK_INTERNAL_1 = 8,
  INNO_LIDAR_MODE_WORK_MAX = 9,
};

enum InnoLidarStatus {
  INNO_LIDAR_STATUS_NONE = 0,
  INNO_LIDAR_STATUS_TRANSITION = 1,
  INNO_LIDAR_STATUS_NORMAL = 2,
  INNO_LIDAR_STATUS_FAILED = 3,
  INNO_LIDAR_STATUS_MAX = 4,
};

enum InnoLidarInFault {
  INNO_LIDAR_IN_FAULT_OTHER = 0,
  INNO_LIDAR_IN_FAULT_POWER_LOW = 1,
  INNO_LIDAR_IN_FAULT_POWER_HIGH = 2,
  INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1 = 3,  // mud 1
  INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE2 = 4,  // mud 2
  INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE3 = 5,  // snow 1
  INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE4 = 6,  // snow 2
  INNO_LIDAR_IN_FAULT_LASER_INTERLOCK = 7,
  INNO_LIDAR_IN_FAULT_COMM_LASER = 8,
  INNO_LIDAR_IN_FAULT_LASER = 9,
  INNO_LIDAR_IN_FAULT_COMM_DSP = 10,
  INNO_LIDAR_IN_FAULT_CANFD_DSP = 11,
  INNO_LIDAR_IN_FAULT_DSP = 12,
  INNO_LIDAR_IN_FAULT_POLYGON_CONTROL = 13,
  INNO_LIDAR_IN_FAULT_POLYGON_SENSOR = 14,
  INNO_LIDAR_IN_FAULT_GALVO_CONTROL = 15,
  INNO_LIDAR_IN_FAULT_GALVO_SENSOR = 16,
  INNO_LIDAR_IN_FAULT_OPTIC1 = 17,
  INNO_LIDAR_IN_FAULT_OPTIC2 = 18,
  INNO_LIDAR_IN_FAULT_IIC_DSP = 19,
  INNO_LIDAR_IN_FAULT_IIC_SOC = 20,
  INNO_LIDAR_IN_FAULT_DSP_EXTWD = 21,
  INNO_LIDAR_IN_FAULT_DBTEMP = 22,
  INNO_LIDAR_IN_FAULT_CHIPTEMP = 23,
  INNO_LIDAR_IN_FAULT_HUMIDITY = 24,
  INNO_LIDAR_IN_FAULT_COMM_ADC = 25,
  INNO_LIDAR_IN_FAULT_FPGACLOCK = 26,
  INNO_LIDAR_IN_FAULT_SOC = 27,
  INNO_LIDAR_IN_FAULT_SOC_EXTWD = 28,
  INNO_LIDAR_IN_FAULT_RAWDATA_STREAM = 29,
  INNO_LIDAR_IN_FAULT_POLYGON_TO = 30,
  INNO_LIDAR_IN_FAULT_GALVO_TO = 31,
  INNO_LIDAR_IN_FAULT_TRIGGER_TO = 32,
  INNO_LIDAR_IN_FAULT_POWSUPL1 = 33,
  INNO_LIDAR_IN_FAULT_POWSUPL2 = 34,
  INNO_LIDAR_IN_FAULT_LPDDR4 = 35,
  INNO_LIDAR_IN_FAULT_FLASH = 36,
  INNO_LIDAR_IN_FAULT_NETWORK1 = 37,
  INNO_LIDAR_IN_FAULT_NETWORK2 = 38,
  INNO_LIDAR_IN_FAULT_OVERHEAT1 = 39,  // want to switch to short range mode
  INNO_LIDAR_IN_FAULT_OVERHEAT2 = 40,  // have to switch to short range mode
  INNO_LIDAR_IN_FAULT_OVERHEAT3 = 41,  // have to stop
  INNO_LIDAR_IN_FAULT_CONFIG1 = 42,
  INNO_LIDAR_IN_FAULT_CONFIG2 = 43,
  INNO_LIDAR_IN_FAULT_ASSERT_FAILURE = 44,
  INNO_LIDAR_IN_FAULT_CPULOAD_HIGH = 45,
  INNO_LIDAR_IN_FAULT_LATENCY_LONG = 46,
  INNO_LIDAR_IN_FAULT_DATA_DROP4 = 47,  // used for stage_signal force job drop
  INNO_LIDAR_IN_FAULT_RAWDATA_TO = 48,
  INNO_LIDAR_IN_FAULT_EXCESSIVE_NOISE = 49,
  INNO_LIDAR_IN_FAULT_DATA_DROP1 = 50,
  INNO_LIDAR_IN_FAULT_DATA_DROP2 = 51,
  INNO_LIDAR_IN_FAULT_DATA_DROP3 = 52,
  INNO_LIDAR_TEMPHIGH_INHIBIT = 53,
  INNO_LIDAR_IN_FAULT_OPTIC1_F = 54,
  INNO_LIDAR_IN_FAULT_REFINTENSITY = 55,
  INNO_LIDAR_IN_FAULT_REPROGRAMMING = 56,
  INNO_LIDAR_IN_FAULT_GALVO_MIRROR = 57,
  INNO_LIDAR_IN_FAULT_MAX_DISTANCE = 58,
  INNO_LIDAR_IN_FAULT_GALVO_OFFSET = 59,
  INNO_LIDAR_IN_FAULT_OPTIC2_F = 60,
  INNO_LIDAR_IN_FAULT_RESERVED16 = 61,
  INNO_LIDAR_IN_FAULT_RESERVED17 = 62,
  INNO_LIDAR_IN_FAULT_RESERVED18 = 63,
  INNO_LIDAR_IN_FAULT_MAX = 64
};

enum InnoTimeSyncConfig {
  INNO_TIME_SYNC_CONFIG_HOST = 0,
  INNO_TIME_SYNC_CONFIG_PTP = 1,
  INNO_TIME_SYNC_CONFIG_GPS = 2,
  INNO_TIME_SYNC_CONFIG_FILE = 3,
  INNO_TIME_SYNC_CONFIG_NTP = 4,
  INNO_TIME_SYNC_CONFIG_MAX = 5,
};

enum InnoTimeSyncType {
  INNO_TIME_SYNC_TYPE_NONE = 0,
  INNO_TIME_SYNC_TYPE_RECORDED = 1,
  INNO_TIME_SYNC_TYPE_HOST = 2,
  INNO_TIME_SYNC_TYPE_GPS_INIT = 3,
  INNO_TIME_SYNC_TYPE_GPS_LOCKED = 4,
  INNO_TIME_SYNC_TYPE_GPS_UNLOCKED = 5,
  INNO_TIME_SYNC_TYPE_PTP_INIT = 6,
  INNO_TIME_SYNC_TYPE_PTP_LOCKED = 7,
  INNO_TIME_SYNC_TYPE_PTP_UNLOCKED = 8,
  INNO_TIME_SYNC_TYPE_FILE_INIT = 9,
  INNO_TIME_SYNC_TYPE_NTP_INIT = 10,
  INNO_TIME_SYNC_TYPE_NTP_LOCKED = 11,
  INNO_TIME_SYNC_TYPE_NTP_UNLOCKED = 12,
  INNO_TIME_SYNC_TYPE_GPS_LOST = 13,
  INNO_TIME_SYNC_TYPE_PTP_LOST = 14,
  INNO_TIME_SYNC_TYPE_NTP_LOST = 15,
  INNO_TIME_SYNC_TYPE_MAX = 16,
};

enum InnoLidarExFault {
  INNO_LIDAR_EX_FAULT_OTHER = 0,
};

enum InnoMessageLevel {
  INNO_MESSAGE_LEVEL_FATAL = 0,
  INNO_MESSAGE_LEVEL_CRITICAL = 1,
  INNO_MESSAGE_LEVEL_ERROR = 2,
  INNO_MESSAGE_LEVEL_TEMP = 3,
  INNO_MESSAGE_LEVEL_WARNING = 4,
  INNO_MESSAGE_LEVEL_DEBUG = 5,
  INNO_MESSAGE_LEVEL_INFO = 6,
  INNO_MESSAGE_LEVEL_TRACE = 7,
  INNO_MESSAGE_LEVEL_DETAIL = 8,
  INNO_MESSAGE_LEVEL_MAX = 9,
};

enum InnoMessageCode {
  INNO_MESSAGE_CODE_NONE = 0,
  INNO_MESSAGE_CODE_LIB_VERSION_MISMATCH,
  INNO_MESSAGE_CODE_READ_TIMEOUT,
  INNO_MESSAGE_CODE_CANNOT_READ,
  INNO_MESSAGE_CODE_BAD_CONFIG_YAML,
  INNO_MESSAGE_CODE_OVERHEAT_PROTECTION,
  INNO_MESSAGE_CODE_TO_NON_WORKING_MODE,
  INNO_MESSAGE_CODE_READ_FILE_END,
  INNO_MESSAGE_CODE_RAW_RECORDING_FINISHED,
  INNO_MESSAGE_CODE_NEW_START,
  INNO_MESSAGE_CODE_ROI_CHANGED,
  INNO_MESSAGE_CODE_GALVO_MIRROR_CHECK_RESULT = 10001,
  INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT = 10002,
};

enum InnoFrameDirection {
  INNO_FRAME_DIRECTION_DOWN = 0,  /* top->bottom   */
  INNO_FRAME_DIRECTION_UP = 1,    /* bottom -> top */
  INNO_FRAME_DIRECTION_MAX = 2,
};

enum InnoItemType {
  INNO_ITEM_TYPE_NONE = 0,
  INNO_ITEM_TYPE_SPHERE_POINTCLOUD = 1,
  INNO_ITEM_TYPE_MESSAGE = 2,
  INNO_ITEM_TYPE_MESSAGE_LOG = 3,
  INNO_ITEM_TYPE_XYZ_POINTCLOUD = 4,
  INNO_ITEM_TYPE_MAX = 5,
};

enum InnoReflectanceMode {
  INNO_REFLECTANCE_MODE_NONE = 0,
  INNO_REFLECTANCE_MODE_INTENSITY = 1,
  INNO_REFLECTANCE_MODE_REFLECTIVITY = 2,
  INNO_REFLECTANCE_MODE_MAX = 3,
};

enum InnoConfidenceLevel {
  INNO_NO_CONFIDENCE = 0,
  INNO_RARE_CONFIDENCE = 1,
  INNO_WELL_CONFIDENCE = 2,
  INNO_FULL_CONFIDENCE = 3,
};

enum InnoMultipleReturnMode {
  /* xxx TODO */
  INNO_MULTIPLE_RETURN_MODE_NONE = 0,
  INNO_MULTIPLE_RETURN_MODE_SINGLE = 1,
  INNO_MULTIPLE_RETURN_MODE_2_STRONGEST = 2,
  // one strongest return and one furthest return
  INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST = 3,
  INNO_MULTIPLE_RETURN_MODE_MAX
};

enum InnoGalvoMode {
  INNO_GALVO_MODE_NONE = 0,
  INNO_GALVO_MODE_NORMAL,
  INNO_GALVO_MODE_FLYBACK,
  INNO_GALVO_MODE_MAX
};

/************
 Simple types
*************/
/* epoch time in micro-sec */
typedef double InnoTimestampUs;

/************
 Constants
*************/
static const uint16_t kInnoMagicNumberDataPacket = 0x176A;
static const uint8_t kInnoMajorVersionDataPacket = 1;
static const uint8_t kInnoMinorVersionDataPacket = 2;
static const uint16_t kInnoMagicNumberStatusPacket = 0x186B;
static const uint8_t kInnoMajorVersionStatusPacket = 1;
static const uint8_t kInnoMinorVersionStatusPacket = 2;

static const uint32_t kInnoDistanceUnitPerMeter = 200;
static const double kMeterPerInnoDistanceUnit =
    1.0 / kInnoDistanceUnitPerMeter;
static const uint32_t kInnoDegreePerPiRad = 180;
static const uint32_t kInnoAngleUnitPerPiRad = 32768;
static const double kRadPerInnoAngleUnit = M_PI / kInnoAngleUnitPerPiRad;
static const double kDegreePerInnoAngleUnit =
    180.0 / kInnoAngleUnitPerPiRad;
static const double kInnoAngleUnitPerDegree =
    kInnoAngleUnitPerPiRad / 180.0;
static const uint32_t kInnoChannelNumberBit = 2;
static const uint32_t kInnoChannelNumber = 1 << kInnoChannelNumberBit;
static const uint32_t kInnoMaxMultiReturn = 2;
static const int16_t kInnoVAngleDiffBase = 196;

static const double kInnoNopROI = 10000.0;

/* 17 bytes per block header */
DEFINE_INNO_COMPACT_STRUCT(InnoBlockHeader) {
  /* horizontal angle, 0 is straight forward, right is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t h_angle;
  /* vertical angle, 0 is the horizon, up is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t v_angle;
  /* relative timestamp (to ts_start_us) in 10us, 0-655,350us */
  uint16_t ts_10us;
  uint16_t scan_idx;     /* point idx within the scan line */
  uint16_t scan_id: 9;   /* id of the scan line */
  // real angle is h_angle + h_angle_diff_1
  int64_t h_angle_diff_1: 9;
  int64_t h_angle_diff_2: 10;
  int64_t h_angle_diff_3: 11;
  // real angle is v_angle + v_angle_diff_1 + kVAngleDiffBase * channel
  int64_t v_angle_diff_1: 8;  // 196 + [-128, 127]
  int64_t v_angle_diff_2: 9;  // 392 + [-256, 255]
  int64_t v_angle_diff_3: 9;  // 588 + [-256, 255]
  /*   0: in sparse region
    0x01: in vertical slow region
    0x10: in horizontal slow region
    0x11: in center ROI */
  uint64_t in_roi: 2;
  uint64_t facet: 3;
  uint64_t reserved_flags: 2; /* all 0 */
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoXyzrD) {
  double x;
  double y;
  double z;
  double radius;
};
DEFINE_INNO_COMPACT_STRUCT_END

/* compact format, 16 + 8 = 24 bytes per point */
DEFINE_INNO_COMPACT_STRUCT(InnoXyzPoint) {
  float x;
  float y;
  float z;
  float radius;
  uint16_t ts_10us;
  uint16_t scan_id: 9;   /* id of the scan line */
  uint16_t in_roi: 2;
  uint16_t facet: 3;
  uint16_t reserved_flags: 2; /* all 0 */
  uint32_t is_2nd_return: 1;
  uint32_t scan_idx: 14;   /* point idx within the scan line */
  uint32_t refl: 9;        /* reflectance, 1-254, 255 means a reflector     */
                           /* or intensity, also 1-254 & 255=reflector      */
  uint32_t type: 2;        /* 0: normal, 1: ground, 2: fog                  */
  uint32_t elongation: 4;  /* elongation */
  uint32_t channel: 2;
  uint16_t ring_id;
};
DEFINE_INNO_COMPACT_STRUCT_END

/* compact format, 4 bytes per point */
DEFINE_INNO_COMPACT_STRUCT(InnoChannelPoint) {
  uint32_t radius: 17;     /* distance in distance unit, range [0, 655.35m] */
  uint32_t refl: 8;        /* reflectance, 1-254, 255 means a reflector     */
                           /* or intensity, also 1-254 & 255=reflector      */
  uint32_t is_2nd_return: 1; /* 0: 1st return, 1: 2nd return                */
  uint32_t type: 2;        /* 0: normal, 1: ground, 2: fog                  */
  uint32_t elongation: 4;  /* elongation */
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoBlock) {
  InnoBlockHeader header;
  InnoChannelPoint points[0];
};
DEFINE_INNO_COMPACT_STRUCT_END

/* 17 + 4 * 4 = 33 bytes */
DEFINE_INNO_COMPACT_STRUCT(InnoBlock1) {
  InnoBlockHeader header;
  InnoChannelPoint points[kInnoChannelNumber];
};
DEFINE_INNO_COMPACT_STRUCT_END

/* 17 + 8 * 4 = 49 bytes */
DEFINE_INNO_COMPACT_STRUCT(InnoBlock2) {
  static inline size_t get_idx(size_t channel, size_t r) {
    /* r0_ch0 r0_ch1 r0_ch2 r0_ch3 r1_ch0 r1_ch1 r1_ch2 r1_ch3 */
    return channel + (r << kInnoChannelNumberBit);
  }
  InnoBlockHeader header;
  InnoChannelPoint points[kInnoChannelNumber * kInnoMaxMultiReturn];
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoMessage) {
  uint32_t size;  // size of the whole InnoMessage,
                  //   i.e. size of content + sizeof(InnoMessage)
  uint32_t src;
  uint64_t id;
  uint32_t level;      /* enum InnoMessageLevel */
  uint32_t code;       /* message code          */
  int32_t reserved[4]; /* all 0                 */
  char content[0];     /* 0 end string          */
};

/*
  Fixed header structure to indicate the firmware/software version.
  This structure won't change during firmware update in the future.
*/
DEFINE_INNO_COMPACT_STRUCT(InnoCommonVersion) {
  /* 2 byte */
  uint16_t magic_number;

  /* 2 byte */
  uint8_t major_version;
  uint8_t minor_version;

  /* 2 byte */
  uint16_t fw_sequence;
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoCommonHeader) {
  /* 6 bytes */
  InnoCommonVersion version;

  /* 4 bytes, cover every thing except checksum */
  uint32_t checksum;

  /* 4 bytes */
  uint32_t size;

  /* 2 bytes */
  uint16_t source_id :4;           /* up to 16 different LiDAR source */
  uint16_t timestamp_sync_type :4; /* enum InnoTimestampSyncType      */
  uint16_t reserved :8;

  /* 8 bytes */
  InnoTimestampUs ts_start_us; /* epoch time of start of frame, in micro-sec */

  /* 2 bytes */
  uint8_t lidar_mode;        /* enum InnoLidarMode    */
  uint8_t lidar_status;      /* enum InnoLidarStatus  */
};
DEFINE_INNO_COMPACT_STRUCT_END

/*
 * Main data packet definition
 *
 * 26 + 12 + 10 + 2 + 4 = 54 bytes, max overhead is 54/1472 = 3.9%,
 *
 * for single-return (27 bytes), 1472 udp payload = 44 blocks,
 * 1.4M points/seconds uses BW = 1.4M / (44 * 4) * 1500 = 11.93MB/s
 *
 * for double-return (43 bytes), 1472 udp payload = 30 blocks
 * 1.4M points/seconds uses BW = 1.4M / (30 * 4) * 1500 = 17.50MB/s
 */
DEFINE_INNO_COMPACT_STRUCT(InnoDataPacket) {
  InnoCommonHeader common;

  /* 12 bytes */
  uint64_t idx;         /* frame index, start from 0                     */
  uint16_t sub_idx;     /* sub-frame index, start from 0 for every frame */
  uint16_t sub_seq;     /* sequence within a sub-frame                   */

  /* 10 byte */
  /* type in enum InnoItemType, each type uses independent global idx */
  uint32_t type :8;
  uint32_t item_number :24;        /* max 4 * 1024 * 1024               */
  uint16_t item_size;              /* max 65535, 0 means variable size  */
  uint32_t topic;                  /* reserved                          */

  /* 2 bytes */
  uint16_t scanner_direction :1; /* 0: top->bottom, 1: bottom->top          */
  uint16_t use_reflectance   :1; /* 0: intensity mode, 1: reflectance mode  */
  uint16_t multi_return_mode :3; /* ... */
  uint16_t confidence_level  :2; /* 0: no confidence, 3: higest             */
  uint16_t is_last_sub_frame :1; /* 1: the last sub frame of a frame        */
  uint16_t is_last_sequence  :1; /* 1: the last piece of a sub frame        */
  uint16_t has_tail :1;          /* has additional tail struct after points */
  uint16_t reserved_flag :6;     /* all 0 */

  /* 4 bytes */
  int16_t roi_h_angle;           /* configured ROI in InnoAngleUnit */
  int16_t roi_v_angle;

  union {
    char c[0];
    InnoBlock1 inno_block1s[0];
    InnoBlock2 inno_block2s[0];
    InnoMessage messages[0];
    InnoXyzPoint xyz_points[0];
  };
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoStatusInFaults) {
  bool no_fault(void) const {
    return faults == 0;
  }
  void clear_faults(void) {
    faults = 0;
  }
  bool no_fault(enum InnoLidarInFault c) const {
    return !((faults >> c) & 1);
  }
  bool no_fault_except(enum InnoLidarInFault c) const {
    return faults == 0 || ((faults & ~(1LU << c)) == 0);
  }
  void set_fault(enum InnoLidarInFault c) {
    faults |= (1LU << c);
  }
  void clear_fault(enum InnoLidarInFault c) {
    faults &= ~(1LU << c);
  }

  /*
   *  Each bit represent one fault defined in enum InnoLidarInFault.
   *  0 means no fault.
   */
  uint64_t faults;
};
DEFINE_INNO_COMPACT_STRUCT_END

DEFINE_INNO_COMPACT_STRUCT(InnoStatusExFaults) {
  bool no_fault(void) const {
    return faults == 0;
  }
  void clear_faults(void) {
    faults = 0;
  }
  bool no_fault(enum InnoLidarExFault c) const {
    return !((faults >> c) & 1);
  }
  bool no_fault_except(enum InnoLidarExFault c) const {
    return faults == 0 || ((faults & ~(1LU << c)) == 0);
  }
  void set_fault(enum InnoLidarExFault c) {
    faults |= (1LU << c);
  }
  void clear_fault(enum InnoLidarExFault c) {
    faults &= ~(1LU << c);
  }

  /*
   *  Each bit represent one fault defined in enum InnoLidarExFault.
   *  0 means no fault.
   */
  uint64_t faults;
};

/*
 * 320 bytes InnoStatusCounters
 */
DEFINE_INNO_COMPACT_STRUCT(InnoStatusCounters) {
  uint64_t point_data_packet_sent;
  uint64_t point_sent;
  uint64_t message_packet_sent;
  uint64_t raw_data_read;
  uint64_t total_frame;
  uint64_t total_polygon_rotation;
  uint64_t total_polygon_facet;
  uint32_t power_up_time_in_second;
  uint32_t process_up_time_in_second;
  uint32_t lose_ptp_sync;
  uint32_t bad_data[4];
  uint32_t data_drop[8];
  uint32_t in_signals[8];
  uint16_t latency_10us_average[6];
  uint16_t latency_10us_variation[6];
  uint16_t latency_10us_max[6];
  uint32_t big_latency_frame;
  uint32_t bad_frame;
  uint32_t big_gap_frame;
  uint32_t small_gap_frame;
  uint16_t cpu_percentage;
  uint16_t mem_percentage;
  uint16_t motor[5];  /* std,min,max1,max2 */
  uint16_t galvo[5];  /* std,min,max1,max2 */
  uint16_t netstat_rx_speed_kBps;
  uint16_t netstat_tx_speed_kBps;
  uint16_t netstat_rx_drop;
  uint16_t netstat_tx_drop;
  uint16_t netstat_rx_err;
  uint16_t netstat_tx_err;
  uint16_t sys_cpu_percentage[4];
  uint32_t lifelong_uptime;
  uint32_t reserved[18];
};

/* 208 bytes InnoStatusSensorReadings */
DEFINE_INNO_COMPACT_STRUCT(InnoStatusSensorReadings) {
  int16_t temperature_fpga_10th_c;
  int16_t temperature_laser_10th_c;
  int16_t temperature_adc_10th_c;
  int16_t temperature_board_10th_c;
  int16_t temperature_det_10th_c[4];
  int16_t temperature_other_10th_c[3];
  uint16_t heater_current_ma;

  uint32_t motor_rpm_1000th;          /* polygon rpm * 1000   */
  uint32_t galvo_fpm_1000th;          /* frame per min * 1000 */
  uint64_t motor_rotation_total;
  uint64_t galvo_round_total;
  uint16_t moisture_index[2];         /* moisture index        */
  uint16_t window_blockage_index[2];  /* window blockage index */
  uint16_t motor[6];  /* ma */
  uint16_t galvo[6];  /* ma */
  uint16_t laser[6];  /* ma */
  uint16_t galvo_status_client;        /* set in client sdk */
  uint16_t galvo_offset_angle_client;  /* set in client sdk */
  uint16_t reserved[48];
};
DEFINE_INNO_COMPACT_STRUCT_END

/*
 * Status packet definition
 */
DEFINE_INNO_COMPACT_STRUCT(InnoStatusPacket) {
  static const uint32_t kSnSize = 16;
  InnoCommonHeader common;

  uint64_t idx;  /* global index of all InnoStatusPacket */

  uint8_t status_packet_interval_ms;  /* status packet send interval in ms    */
  uint8_t pre_lidar_mode;             /* previous InnoLidarMode               */
  uint16_t in_transition_mode_ms;  /* time (ms), LiDAR in the transition mode */

  char sn[kSnSize];                /* lidar serial number */
  uint16_t fault_version;
  uint16_t reserve;
  uint32_t reserved[3];

  InnoStatusInFaults in_faults;
  InnoStatusExFaults ex_faults;
  InnoStatusCounters counters;
  InnoStatusSensorReadings sensor_readings;
};
DEFINE_INNO_COMPACT_STRUCT_END


/*
 * Raw4 packet definition
 */
struct InnoRaw4Packet {
  uint32_t idx;

  enum RawPacketType {
    TYPE_SN = 0,
    TYPE_CAUSE = 1,
    TYPE_RAWDATA = 2,
  } field_type;

  bool is_field_last;

  const char* buffer;
  int buffer_size;

  bool is_message_last() const { return field_type == TYPE_RAWDATA; }
};


/*
 * Raw4 udp header
 */
struct Raw4UdpHeader {
  // net size
  static const int kHeaderSize = 10;

  // offset
  static const uint16_t kIdxOffset = 0x0;
  static const uint16_t kFieldTypeOffset = 0x4;
  static const uint16_t kFieldSeqIdOffset = 0x5;
  static const uint16_t kFlagOffset = 0x9;

  // flag
  static const uint16_t kFlagFieldEnd = 0x1;

  // member
  uint32_t idx;
  uint8_t field_type;
  uint32_t field_sequence_id;
  uint8_t flag;

  bool is_field_end() const { return flag == kFlagFieldEnd; }
  void set_field_end() { flag = kFlagFieldEnd; }
};

#endif  // SDK_COMMON_INNO_LIDAR_PACKET_H_
