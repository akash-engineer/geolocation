/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <unistd.h>
#include <linux/limits.h>
#include <mcheck.h>

#include "src/sdk_common/inno_lidar_api.h"
#include "src/utils/inno_lidar_log.h"
#include "pcs/command_parser.h"
#include "pcs/pcs.h"
#include "pcs/version.h"

namespace innovusion {

static void show_version() {
  inno_log_info("VERSION: %s", innovusion_version);
  inno_log_info("BUILD_TAG: %s", innovusion_build_tag);
  inno_log_info("BUILD_TIME: %s", innovusion_build_time);
  inno_log_info("API: %s", inno_api_version());
  inno_log_info("API_BUILD_TAG: %s", inno_api_build_tag());
  inno_log_info("API_BUILD_TIME: %s", inno_api_build_time());
}

}  // namespace innovusion

static void launch_viewer(const innovusion::CommandParser &cmd_parser) {
  if (!cmd_parser.show_viewer || !cmd_parser.ws_enabled()) {
    return;
  }
  char cmd[PATH_MAX + 1024];
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  std::string path = std::string(result, (count > 0) ? count : 0);
  size_t found = path.find_last_of("/\\");
  path = path.substr(0, found);
  int r = snprintf(cmd, sizeof(cmd),
                   "%s/launch-viewer.bash 127.0.0.1 %hu",
                   path.c_str(),
                   cmd_parser.tcp_port);
  inno_log_verify(r < static_cast<int>(sizeof(cmd)), "buffer is too small");
  inno_log_info("about to run: %s", cmd);
  r = system(cmd);
  inno_log_info("cmd issued, ret: %d", r);
}

int main(int argc, char *argv[]) {
  // show version
  innovusion::show_version();

  // step1: parse
  innovusion::CommandParser cmp = innovusion::CommandParser(argc, argv);
  // step2: check and config Log
  // set up logs without callback
  if (cmp.debug_level > INNO_LOG_LEVEL_INFO) {
    mtrace();
  }
  inno_lidar_set_logs(cmp.quiet ? -1 : 1,
                      cmp.quiet ? -1 : 2,
                      cmp.log_filename.c_str(),
                      cmp.log_file_rotate_number,
                      cmp.log_file_max_size_k * 1000UL,
                      NULL,  // callback NULL
                      NULL,  // this NULL
                      cmp.error_log_filename.c_str(),
                      cmp.error_log_file_rotate_number,
                      cmp.error_log_file_max_size_k * 1000UL,
                      1 /*use async log*/);
  // step 3: set level
  inno_lidar_set_log_level(cmp.debug_level);

  // launch view
  launch_viewer(cmp);

  int64_t run_time_ms = -1;
  if (!cmp.run_time_s.empty()) {
    char *end;
    run_time_ms = strtol(cmp.run_time_s.c_str(), &end, 10) * 1000;
    if (end != cmp.run_time_s.c_str() + cmp.run_time_s.length()) {
      inno_log_error("--run_time_s %s parameter is invalid",
                     cmp.run_time_s.c_str());
      exit(1);
    }
  }
  uint64_t start_time_ms = innovusion::InnoUtils::
      get_time_ns(CLOCK_MONOTONIC_RAW) / 1000 / 1000;

  for (int i = 0; ; i++) {
    innovusion::PCS *pcs = new innovusion::PCS(cmp);
    inno_log_verify(pcs, "pcs");
    pcs->run();
    inno_log_info("run #%d stopped.", i);

    uint64_t cur_time_ms = innovusion::InnoUtils::
        get_time_ns(CLOCK_MONOTONIC_RAW) / 1000 / 1000;

    // run_time_ms <= 0 means pcs keep running
    if (run_time_ms > 0 &&
       (int64_t)(cur_time_ms - start_time_ms) > run_time_ms) {
      delete pcs;
      break;
    }

    if (cmp.lidar.is_local_source() && cmp.retry_remote < 0) {
      delete pcs;
      break;
    }

    if (i >= cmp.retry_remote && cmp.retry_remote >= 0) {
      delete pcs;
      break;
    }
    delete pcs;
    sleep(1);
  }
  return 0;
}
