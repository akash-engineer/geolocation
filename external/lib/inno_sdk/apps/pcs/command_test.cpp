/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/command_test.h"

#include <utility>
#include "pcs/pc_server_ws_processor.h"

namespace innovusion {
CommandTest::CommandTest(PCS *pcs, const std::string& command,
                         const std::string& interval,
                         const std::string& run_time_s) {
  // init properties
  inno_log_verify(pcs, "pcs");
  inno_log_verify(!command.empty(), "command cannot be empty here");
  this->pcs_ = pcs;
  this->command_ = command;

  if (interval.empty()) {
    usage_("--test-command-interval-ms must be specified");
    exit(1);
  }
  char *end;
  this->send_interval_ms_ = strtol(interval.c_str(), &end, 10);
  if (end != interval.c_str() + interval.length()) {
    usage_("--test-command-interval-ms parameter is invalid:" + interval);
    exit(1);
  }

  if (send_interval_ms_ < kMinIntervalMs_) {
    usage_("--test-command-interval-ms " + interval + " is too small, "
           "should more than " + std::to_string(kMinIntervalMs_));
    exit(1);
  }

  if (!run_time_s.empty()) {
    this->run_time_ms = strtol(run_time_s.c_str(), &end, 10);
    if (end != run_time_s.c_str() + run_time_s.length()) {
      usage_("--run_time_s %s parameter is invalid:" + run_time_s);
      exit(1);
    }
    this->run_time_ms *= 1000;
  }
}

CommandTest::~CommandTest() {
  //
}

#define GET_PRIFIX "get_"
#define SET_PRIFIX "set_"
void *CommandTest::loop_() {
  uint64_t start_time_us = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW) / 1000;
  std::string name, ori_value;
  ServerWsProcessor::split_query_string_s_(command_, &name, &ori_value);
  int ret = 0;
  uint64_t target_us = send_interval_ms_ * 1000;
  while (!pcs_->get_command_test_thread()->has_shutdown()) {
    uint64_t time_s1 = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    inno_log_info("start exec command:%s", command_.c_str());
    // send command
    std::string value = get_value(name, ori_value);
    if (InnoUtils::start_with(name.c_str(), GET_PRIFIX)) {
      std::string pure_cmd = name.substr(sizeof(GET_PRIFIX) - 1);
      std::string result;
      ret = pcs_->get_pcs(pure_cmd,
                          value,
                          &result,
                          nullptr,
                          true);
    } else if (InnoUtils::start_with(name.c_str(), SET_PRIFIX)) {
      std::string pure_cmd = name.substr(sizeof(SET_PRIFIX) - 1);
      ret = pcs_->set_pcs(pure_cmd,
                          value,
                          nullptr);
    } else {
      inno_log_panic("unsupported command:%s", command_.c_str());
    }
    if (ret) {
      inno_log_panic("command return %d", ret);
    }
    uint64_t time_s2 = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    uint64_t time_s2_us = time_s2 / 1000;
    // run_time_ms <= 0 means no limit
    if (run_time_ms > 0 &&
        (int64_t)(time_s2_us - start_time_us) > run_time_ms * 1000) {
      pcs_->stop();
    }
    uint64_t elapsed_us = (time_s2 - time_s1) / 1000;
    if (target_us > elapsed_us) {
      usleep(target_us - elapsed_us);
    }
  }
  return nullptr;
}

void CommandTest::usage_(const std::string& err) {
  fprintf(stderr,
          "%s\n"
          "Usage of test command:\n"
          "[--test-command <TEST_COMMAND>] \t\t\t\t|"
          "Specify which command you want to test, the command will be "
          "executed periodically. If you set value to 'random', it means"
          "the param will be generated randomly every time. "
          "e.g. set_roi=random . "
          "Commands support random param value:%s\n"
          "[--run-time-s <RUN_TIME_IN_SECONDS>] \t\t\t\t|"
          "program will exit after running RUN_TIME_IN_SECONDS. "
          "RUN_TIME_IN_SECONDS <= 0 means do not exit.\n"
          "[--test-command-interval-ms <TEST_COMMAND_INTERVAL_IN_MS>] \t|"
          "do command specified by --test-command periodically and "
          "period is TEST_COMMAND_INTERVAL_IN_MS\n",
          err.c_str(),
          "[set_roi]");
}

void *CommandTest::start(void *ctx) {
  inno_log_verify(ctx, "ctx of CommandTest is not set");
  auto *ct = reinterpret_cast<CommandTest *>(ctx);
  ct->loop_();
  return nullptr;
}

std::string CommandTest::get_value(const std::string& name,
                                   const std::string& value) {
  std::string true_val;
  if (strcmp(value.c_str(), "random") == 0) {
    // generate random value
    if (name.find("set_") == 0) {
      if (name == "set_roi") {
        unsigned int seed = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
        uint32_t r = rand_r(&seed);
        double h_roi = kMinHRoi_ + (r % 100) * (kMaxHRoi_ - kMinHRoi_) / 100;
        r = rand_r(&seed);
        double v_roi = kMinVRoi_ + (r % 100) * (kMaxVRoi_ - kMinVRoi_) / 100;
        true_val = std::to_string(h_roi) + "," + std::to_string(v_roi);
      } else {
        inno_log_panic("command '%s' does not support random param",
                       name.c_str());
      }
    } else {
      inno_log_panic("can not set random param for command:%s", name.c_str());
    }
  } else {
    true_val = value;
  }
  return true_val;
}

}  // namespace innovusion

