/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_MATH_TABLES_H_
#define UTILS_MATH_TABLES_H_

#include <stddef.h>
#include <stdint.h>

#include <math.h>

#include "./log.h"

// #define EXACT_SIN_COS
#define EXACT_ASIN_ATAN

namespace innovusion {

class MathTables {
 private:
  // must be same as defined in sdk/inno_lidar_packet.h
  static const size_t kInnoAngleUnitPerPiRad = 32768;
  static constexpr double kRadPerInnoAngleUnit = M_PI / kInnoAngleUnitPerPiRad;

  static const int32_t kDegreeInPi = 180;
  static constexpr double kInnoAngleUnitPerOneDegree =
    kInnoAngleUnitPerPiRad / static_cast<double>(kDegreeInPi);
  static const int32_t kAngleTableSize = 2 * kInnoAngleUnitPerPiRad;

  static const int32_t kASinTableSize = 10000;
  static const int32_t kATanTableSize = 45000;
  static const int32_t kAtanTableScale = 10000;
  static const int32_t kAsinTableScale = 10000;

 private:
#ifndef EXACT_SIN_COS
  static float sin_table_[];
  static float cos_table_[];
#endif

#ifndef EXACT_ASIN_ATAN
  static int32_t asin_table_[];
  static int32_t atan_table_[];
#endif

 public:
  inline static void verify_unit(size_t unit) {
    inno_log_verify(unit == kInnoAngleUnitPerPiRad,
                    "unit doesn't match %" PRI_SIZEU
                    " vs %" PRI_SIZEU "",
                    unit, kInnoAngleUnitPerPiRad);
  }
  inline static double degree_to_rad(double v) {
    return v * M_PI / kDegreeInPi;
  }

  inline static double get_scaled_theta(double theta) {
    if (theta < 0) {
      int32_t multiple = 1 + (static_cast<int>(theta) / -360);
      theta += 360 * multiple;
    }
    if (theta >= 360) {
      int32_t multiple = static_cast<int32_t>(theta) / 360;
      theta -= 360 * multiple;
    }
    return theta * kInnoAngleUnitPerOneDegree;
  }

  inline static double lookup_sin_table_in_unit_exact(int i) {
    return sin(i * kRadPerInnoAngleUnit);
  }

  inline static double lookup_sin_table_in_unit(int i) {
#ifdef EXACT_SIN_COS
    return lookup_sin_table_in_unit_exact(i);
#else
    return sin_table_[i];
#endif
  }

  inline static double lookup_cos_table_in_unit_exact(int i) {
    return cos(i * kRadPerInnoAngleUnit);
  }

  inline static double lookup_cos_table_in_unit(int i) {
#ifdef EXACT_SIN_COS
    return lookup_cos_table_in_unit_exact(i);
#else
    return cos_table_[i];
#endif
  }

  inline static double lookup_sin_table_exact(double theta) {
    return sin(degree_to_rad(theta));
  }

  inline static double lookup_sin_table(double theta) {
#ifdef EXACT_SIN_COS
    return lookup_sin_table_exact(theta);
#else
    double scaled_theta;
    int32_t truncated_theta;

    scaled_theta = get_scaled_theta(theta);
    truncated_theta = static_cast<int32_t>(scaled_theta);
#if 0
    if (truncated_theta < 0 || truncated_theta >= kStepsInOneDegree * 360) {
      inno_log_error("bad angle %f %d", theta, truncated_theta);
      truncated_theta = 0;
    }
#endif
    return sin_table_[truncated_theta] + (scaled_theta - truncated_theta) *
    (sin_table_[truncated_theta + 1] - sin_table_[truncated_theta]);
#endif
  }

  inline static double lookup_cos_table_exact(double theta) {
    return cos(degree_to_rad(theta));
  }

  inline static double lookup_cos_table(double theta) {
#ifdef EXACT_SIN_COS
    return lookup_cos_table_exact(theta);
#else
    double scaled_theta;
    int32_t truncated_theta;

    scaled_theta = get_scaled_theta(theta);
    truncated_theta = static_cast<int32_t>(scaled_theta);
#if 0
    if (truncated_theta < 0 || truncated_theta >= kStepsInOneDegree * 360) {
      inno_log_error(stderr, "bad angle %f %d", theta, truncated_theta);
      truncated_theta = 0;
    }
#endif
    return cos_table_[truncated_theta] + (scaled_theta - truncated_theta) *
        (cos_table_[truncated_theta + 1] - cos_table_[truncated_theta]);
#endif
  }

  inline static int lookup_atan_table_exact(double value, int32_t *result) {
    *result = static_cast<int>(atan(value) / kRadPerInnoAngleUnit);
    return 0;
  }

  inline static int lookup_atan_table(double value, int32_t *result) {
#ifdef EXACT_ASIN_ATAN
    return lookup_atan_table_exact(value, result);
#else
    int atan_table_index = static_cast<int>(value * kAtanTableScale +
                                            kATanTableSize);
    if (atan_table_index < 0 || atan_table_index >= 2 * kATanTableSize) {
      inno_log_error("atan_table_index: %d out of bound", atan_table_index);
      return -1;
    } else {
      *result = atan_table_[atan_table_index];
      return 0;
    }
#endif
  }

  inline static int lookup_asin_table_exact(double value, int32_t *result) {
    *result = static_cast<int>(asin(value) / kRadPerInnoAngleUnit);
    return 0;
  }

  inline static int lookup_asin_table(double value, int32_t *result) {
#ifdef EXACT_ASIN_ATAN
    return lookup_asin_table_exact(value, result);
#else
    int asin_table_index = static_cast<int>(value * kAsinTableScale +
                                            kASinTableSize);
    if (asin_table_index < 0 || asin_table_index >= 2 * kASinTableSize) {
      inno_log_error("asin_table_index: %d out of bound", asin_table_index);
      return -1;
    } else {
      *result = asin_table_[asin_table_index];
      return 0;
    }
#endif
  }

 public:
  static class TablesInit {
   public:
    TablesInit(double inno_angle_unit) {
      setup_table_(inno_angle_unit);
    }

   private:
    void setup_table_(double inno_angle_unit);
  } tables_init;
};

}  // namespace innovusion

#endif  // UTILS_MATH_TABLES_H_

