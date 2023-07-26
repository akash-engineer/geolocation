/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_TYPES_CONSTS_H_
#define UTILS_TYPES_CONSTS_H_

#include <stddef.h>
#include <stdint.h>

namespace innovusion {
/* internal typedef */

typedef double InnoEpSecondDouble;
typedef int64_t InnoEpMs;
typedef int64_t InnoEpUs;
typedef int64_t InnoEpNs;

typedef int64_t InnoFpgaNs;
/* xxx todo: up to 388 days from power up */
typedef int64_t InnoFpgaSubNs;

/* 
 * 2 ^ 31 / 1000 / 1000 / 32 = 36ms
 * max range is -72ms to 72ms
 */
typedef int32_t InnoSubNsOffset;

/*
 * 2 ^ 15 / 32 = 1024ns
 * max range is -1024ns to 1023ns
 */
typedef int16_t InnoSubNsOffset16;

/* internal consts */
class InnoConsts {
 public:
  // unit
  static const size_t kUsInSecond = 1000 * 1000;
  static const size_t kNsInSecond = 1000 * 1000 * 1000;
  static const size_t kSecondInMinute = 60;
  static const size_t kNsIn10Us = 10 * 1000;

  // inno unit
  static const size_t kSubNsBits = 5;
  static const size_t kSubNsInNs = 1 << kSubNsBits;

  // physics
  static const size_t kSpeedOfLight = 299792458;  // meter per second
  static const double kAirRefractiveIndex;
  static const double kLightTraveMeterPerSubNs;

  // invalid
  static const InnoFpgaNs kInvalidInnoFpgaNs = 0;
  static const InnoFpgaSubNs kInvalidInnoFpgaSubNs = 0;

  // lidar
  static const size_t kMaxTriggerPerSecond = 2000 * 1000;
  static const size_t kMinTriggerPerSecond = 200 * 1000;
  static const size_t kMinPolygonRPM = 3000;
  static const size_t kPolygonFacet = 5;
  static const size_t kMaxTrigger = 5;
  static const size_t kMinFPS = 5;
  static const size_t kGalvoEncoderHz = 50 * 1000;  // xxx todo: ask haosen
  static const size_t KNormalRPM = 4800;
  static const size_t KShortRangeRPM = 2400;
  static const size_t KCalibrationRPM = 5362;
  static const size_t kMaxRPM = 12000;
  static const InnoFpgaSubNs kMinPolygonPeriodSubNs =
    kSecondInMinute / 7000.0 * kNsInSecond * kSubNsInNs;
  static const InnoFpgaSubNs kMaxPolygonPeriodSubNs =
    kSecondInMinute / 2000.0 * kNsInSecond * kSubNsInNs;
  static const InnoFpgaSubNs kShortRangePolygonPeriodSubNs =
    kSecondInMinute * kNsInSecond * kSubNsInNs / KShortRangeRPM;
  static const InnoFpgaSubNs kNormalPolygonPeriodSubNs =
    kSecondInMinute * kNsInSecond * kSubNsInNs / KNormalRPM;
  static const InnoFpgaSubNs kCalibrationPolygonPeriodSubNs =
    kSecondInMinute * kNsInSecond * kSubNsInNs / KCalibrationRPM;
  static const int32_t kGalvoEncoderMin = 0x10000;  // for galvo min angle
  static const int32_t kGalvoEncoderMax = 0;        // for galvo max angle
  // for ring id, fps is 10 at least
  static const size_t kMaxScanLinePerFrame = kMaxRPM * kPolygonFacet / 60 / 10;
  static const size_t kGalvoPeridUnitsPerSecond = 15000;
};

/* internal converters */
class InnoConverts {
 public:
  static inline InnoFpgaSubNs ns_to_sub_ns(InnoFpgaNs t) {
    return t << InnoConsts::kSubNsBits;
  }
  static inline InnoFpgaNs sub_ns_to_ns(InnoFpgaSubNs t) {
    return t >> InnoConsts::kSubNsBits;
  }
  static inline InnoEpUs ns_to_us(InnoFpgaNs t) {
    return t / 1000;
  }
};

enum InnoBadFpgaData {
  INNO_BAD_FPGA_DATA_NORMAL = 0,
  INNO_BAD_FPGA_DATA_INVALID_TYPE = 100,
  INNO_BAD_FPGA_DATA_INVALID_SUB_TYPE = 101,
  INNO_BAD_FPGA_DATA_INVALID_RECEIVE_1st = 111,
  INNO_BAD_FPGA_DATA_INVALID_RECEIVE_2nd = 112,
  INNO_BAD_FPGA_DATA_INVALID_RECEIVE_3rd = 113,
  INNO_BAD_FPGA_DATA_MAX,
};

}  // namespace innovusion

#endif  // UTILS_TYPES_CONSTS_H_
