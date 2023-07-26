/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_COMMAND_TEST_H_
#define PCS_COMMAND_TEST_H_

#include <string>
#include <utility>
#include "src/utils/inno_thread.h"
#include "pcs/pcs.h"

namespace innovusion {
class PCS;
class CommandTest {
 private:
  static constexpr uint32_t kMinIntervalMs_ = 100;
  static constexpr double kMinVRoi_ = -25.0;
  static constexpr double kMaxVRoi_ = 25.0;
  static constexpr double kMinHRoi_ = -60.0;
  static constexpr double kMaxHRoi_ = 60.0;

 public:
  CommandTest(PCS *pcs, const std::string& command,
              const std::string& interval, const std::string& run_time_ms);
  ~CommandTest();
  static void *start(void * ctx);
  static std::string get_value(const std::string& name,
                               const std::string& value);

 private:
  void *loop_();
  static void usage_(const std::string& err);

 public:
  int64_t run_time_ms{-1L};

 private:
  PCS *pcs_;
  std::string command_;
  uint16_t send_interval_ms_;
};

}  // namespace innovusion

#endif  // PCS_COMMAND_TEST_H_
