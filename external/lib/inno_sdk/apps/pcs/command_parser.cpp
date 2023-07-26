/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/command_parser.h"

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>

#include "src/sdk_common/inno_lidar_api.h"
#include "src/utils/inno_lidar_log.h"
#include "src/utils/utils.h"

namespace innovusion {
LidarCommandConfig::LidarCommandConfig() {
  lidar_port = kDefaultServerLidarPort_;
  lidar_udp_port = -1;
  file_speed = 10000;  // x1 play speed
  file_rewind = 0;
  file_skip = 0;
  processed = 0;
  use_xyz = 0;
  set_falcon_eye = false;
  roi_center_h = 0;
  roi_center_v = 0;
  reflectance = INNO_REFLECTANCE_MODE_REFLECTIVITY;
  multireturn = INNO_MULTIPLE_RETURN_MODE_SINGLE;
  lidar_id = 0;
  lidar_port_is_default_ = true;
}

int LidarCommandConfig::check() const {
  if (lidar_ip.empty() && data_filename.empty()) {
    fprintf(stderr, "Please specify Lidar IP or data file.\n");
    return -1;
  }
  /*
  if (lidar_ip.empty() && yaml_filename.empty() && processed == 0) {
    fprintf(stderr, "Please specify Lidar YAML file.\n");
    return -1;
  }
  */
  if (multireturn <= INNO_MULTIPLE_RETURN_MODE_NONE ||
      multireturn >= INNO_MULTIPLE_RETURN_MODE_MAX) {
    fprintf(stderr, "--multireturn must either be 1 (single strongest) "
            "or 2 (2 strongest).\n");
    return -1;
  }
  if (reflectance <= INNO_REFLECTANCE_MODE_NONE ||
      reflectance >= INNO_REFLECTANCE_MODE_MAX) {
    fprintf(stderr, "--reflectance must either be 1 (intensity) "
            "or 2 (reflectivity).\n");
    return -1;
  }
  return 0;
}

CommandParser::CommandParser(int argc, char *argv[])
    : lidar() {
  argc_ = argc;
  argv_ = argv;

  for (int i = 0; i < argc; i++) {
    if (i != 0) {
      full_command_line += " ";
    }
    full_command_line += std::string(argv[i]);
  }
  retry_remote = -1;  // retry forever
  // add a initial value "0.0.0.0" for udp_client_ip parameter
  udp_client_ip = kInvalidIpAddress;
  udp_port_data = kDefaultUdpPort_;
  udp_port_status = kDefaultUdpPort_;
  udp_port_raw = 0;
  udp_port_status_local = 0;
  udp_port_message = kDefaultUdpPort_;
  tcp_port = kDefaultServerTcpPort_;
  tcp_port_is_default_ = true;

  status_interval_ms = 50;
  record_inno_pc_size_in_m = 0;
  inno_pc_record_npy = 0;
  record_raw_size_in_m = 0;
  debug_level = INNO_LOG_LEVEL_INFO;
  log_filename = std::string(argv[0]) + ".log";
  log_file_rotate_number = 3;
  log_file_max_size_k = 10000;
  error_log_filename = std::string(argv[0]) + ".log.err";
  error_log_file_rotate_number = 2;
  error_log_file_max_size_k = 1000;

  rosbag_size_in_m = 0;

  get_version = false;
  show_viewer = 0;
  quiet = 0;
  sleep_second = 0;

  parse_parameter_(argc, argv);
}

CommandParser::~CommandParser() {
}

bool CommandParser::ws_enabled() const {
  return tcp_port > 0;
}

bool CommandParser::is_client_mode() const {
  return lidar.processed;
}

void CommandParser::usage_() {
  // https://stackoverflow.com/questions/9725675/is-there-a-standard-format-for-command-line-shell-help-text
  fprintf(stderr,
          "usage: %s \n"
          "\t{--lidar-ip <INPUT_LIDAR_IP> "
          "[--lidar-port <INPUT_LIDAR_TCP_PORT> "
          "--lidar-udp-port <INPUT_LIDAR_UDP_PORT>] "
          "[--retry <RETRY_COUNT>] |\n"
          "\t --file <INPUT_DATA_FILE> [--yaml <INPUT_YAML_FILE>] "
          "[--speed <PLAY_SPEED>] [--rewind <REWIND_TIMES>] "
          "[--skip <SKIP_IN_MB>]}\n"
          "\t[--falcon-eye <X>,<Y>]\n"
          "\t[--reflectance <REFLECTANCE_MODE>]\n"
          "\t[--multireturn <MULTI_RETURN_MODE>]\n"
          "\t[--lidar-id <LIDAR_ID>]\n"
          "\t[--udp-ip <UDP_DEST_IP> [--udp-port <DATA_UDP_DEST_PORT>]\n"
          "\t  [--udp-port-status <STATUS_UDP_DEST_PORT>]\n"
          "\t  [--udp-port-message <MESSAGE_UDP_DEST_PORT>]]\n"
          "\t  [--udp-port-raw <RAW_UDP_DEST_PORT>]]\n"
          "\t[--udp-port-status-local <LOCAL_STATUS_UDP_DEST_PORT>]\n"
          "\t[--tcp-port <TCP_LISTEN_PORT>]\n"
          "\t[--status-interval-ms <INTERVAL_IN_MS>]\n"
          "\t[--record-inno-pc-filename <RECORD_INNO_PC_FILE>\n"
          "\t  [--record-inno-pc-size-in-m <RECORD_INNO_PC_FILE_SIZE>]\n"
          "\t  [--inno-pc-record-npy]]\n"
          "\t[--record-png-filename <RECORD_PNG_FILE>\n"
          "\t[--record-rosbag-filename <RECORD_ROSBAG_FILE>\n"
          "\t  [--record-rosbag-size-in-m <RECORD_ROSBAG_SIZE>]]\n"
          "\t[--record-raw-filename <RECORD_RAW_FILE>\n"
          "\t  [--record-raw-size-in-m <RECORD_RAW_FILE_SIZE>]]\n"
          "\t[--config <CONFIG_FILE>]\n"
          "\t[--config2 <CONFIG_FILE2>]\n"
          "\t[--dtc <DTC_FILE>]\n"
          "\t[--out-format <OUT_FORMAT>]\n"
          "\t[--processed]\n"
          "\t[--debug <DEBUG_LEVEL>] [-v] [-h] [--quiet]\n"
          "\t[--log-filename <LOG_FILE>] "
          "[--log-file-rotate-number <ROTATE_NUMBER>] "
          "[--log-file-max-size-k <MAX_LOG_FILE_SIZE_IN_K>]\n"
          "\t[--error-log-filename <ERROR_LOG_FILE>] "
          "[--error-log-file-rotate-number <ROTATE_NUMBER>] "
          "[--error-log-file-max-size-k <MAX_ERROR_LOG_FILE_SIZE_IN_K>]\n"
          "\t[--show-viewer]\n"
          "For the --falcon-eye option,\n"
          "\tx: ROI horizontal center (in [-60, 60] degrees)\n"
          "\ty: ROI vertical center (in [-25, 25], degrees)\n"
          "\t[--test-command <TEST_COMMAND>] "
          "[--run-time-s <RUN_TIME_IN_SECONDS>] "
          "[--test-command-interval-ms <TEST_COMMAND_INTERVAL_IN_MS>]\n"
          ,
          argv_[0]);
}

void CommandParser::parse_parameter_(int argc, char *argv[]) {
  /* getopt_long stores the option index here. */
  int c;
  static struct option long_options[] = {
    /* These options set a flag. */
    {"lidar-ip", required_argument, 0, 'n'},
    {"lidar-port", required_argument, 0, 'P'},
    {"lidar-udp-port", required_argument, 0, 'O'},
    {"retry", required_argument, 0, 'T'},
    {"file", required_argument, 0, 'f'},
    {"yaml", required_argument, 0, 'y'},
    {"speed", required_argument, 0, 's'},
    {"rewind", required_argument, 0, 'd'},
    {"skip", required_argument, 0, 'S'},
    {"processed", no_argument, &lidar.processed, 1},
    {"falcon-eye", required_argument, 0, 'e'},
    {"reflectance", required_argument, 0, 'F'},
    {"multireturn", required_argument, 0, 'M'},
    {"lidar-id", required_argument, 0, 'i'},
    {"udp-ip", required_argument, 0, 'u'},
    {"udp-port", required_argument, 0, 'U'},
    {"udp-port-status", required_argument, 0, 'W'},
    {"udp-port-message", required_argument, 0, 'w'},
    {"udp-port-raw", required_argument, 0, 'k'},
    {"udp-port-status-local", required_argument, 0, 'l'},
    {"tcp-port", required_argument, 0, 'p'},
    {"status-interval-ms", required_argument, 0, 'I'},
    {"record-inno-pc-filename", required_argument, 0, 'o'},
    {"record-inno-pc-size-in-m", required_argument, 0, 'm'},
    {"inno-pc-record-npy", no_argument, &inno_pc_record_npy, 1},
    {"record-png-filename", required_argument, 0, 't'},
    {"record-rosbag-filename", required_argument, 0, 'b'},
    {"record-rosbag-size-in-m", required_argument, 0, 'B'},
    {"record-raw-filename", required_argument, 0, 'r'},
    {"record-raw-size-in-m", required_argument, 0, 'R'},
    {"config", required_argument, 0, 'g'},
    {"config2", required_argument, 0, 'G'},
    {"dtc", required_argument, 0, 'H'},
    {"use-xyz", required_argument, 0, 'x'},
    {"debug", required_argument, 0, 'D'},
    {"log-filename", required_argument, 0, 'N'},
    {"log-file-rotate-number", required_argument, 0, 'a'},
    {"log-file-max-size-k", required_argument, 0, 'A'},
    {"error-log-filename", required_argument, 0, 'C'},
    {"error-log-file-rotate-number", required_argument, 0, 'j'},
    {"error-log-file-max-size-k", required_argument, 0, 'J'},
    {"show-viewer", no_argument, &show_viewer, 1},
    {"quiet", no_argument, &quiet, 1},
    {"run-time-s", required_argument, 0, 'K'},
    {"test-command", required_argument, 0, 'L'},
    {"test-command-interval-ms", required_argument, 0, 'X'},
    {"sleep", required_argument, 0, 'E'},
    {"help", no_argument, NULL, 'h'},
    {0, 0, 0, 0}
  };
  const char *optstring = "a:b:cd:e:f:g:hi:j:k:l:m:n:o:p:qr:s:t:u:vw:x:y:"
                          "A:B:C:D:E:F:G:H:I:J:K:L:M:N:O:P:R:S:T:U:VW:X:Y";
  while (1) {
    int option_index = 0;
    c = getopt_long(argc, argv, optstring, long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1) {
      break;
    }

    switch (c) {
      case 0:
        /* If this option set a flag, do nothing else now. */
        if (long_options[option_index].flag != 0) {
          break;
        }
        inno_log_verify(optarg == NULL, "option %s with arg %s",
                        long_options[option_index].name, optarg);
        break;

      case 'h':
        usage_();
        exit(0);
        break;

      case 'n':
        lidar.lidar_ip = optarg;
        break;

      case 'P':
        lidar.lidar_port = strtoul(optarg, NULL, 0);
        lidar.lidar_port_is_default_ = false;
        break;

      case 'O':
        lidar.lidar_udp_port = strtoul(optarg, NULL, 0);
        break;

      case 'T':
        retry_remote = atoi(optarg);
        break;

      case 'f':
        lidar.data_filename = optarg;
        break;

      case 'y':
        lidar.yaml_filename = optarg;
        break;

      case 's':
        lidar.file_speed = atoi(optarg);
        break;

      case 'd':
        lidar.file_rewind = atoi(optarg);
        break;

      case 'S':
        lidar.file_skip = atol(optarg);
        break;

      case 'c':
        lidar.processed = 1;
        break;

      case 'e':
        if (sscanf(optarg, "%lf,%lf",
                   &lidar.roi_center_h,
                   &lidar.roi_center_v) == 2) {
          lidar.set_falcon_eye = true;
        } else {
          inno_log_error("invalid --falcon-eye option %s",
                         optarg);
          exit(1);
        }
        break;

      case 'F':
        lidar.reflectance = (enum InnoReflectanceMode)atoi(optarg);
        break;

      case 'M':
        lidar.multireturn = (enum InnoMultipleReturnMode)atoi(optarg);
        break;

      case 'i':
        lidar.lidar_id = strtoul(optarg, NULL, 0);
        break;

      case 'u':
        udp_client_ip = optarg;
        break;

      case 'U':
        udp_port_data = strtoul(optarg, NULL, 0);
        break;

      case 'W':
        udp_port_status = strtoul(optarg, NULL, 0);
        break;

      case 'k':
        udp_port_raw = strtoul(optarg, NULL, 0);
        break;

      case 'l':
        udp_port_status_local = strtoul(optarg, NULL, 0);
        break;

      case 'w':
        udp_port_message = strtoul(optarg, NULL, 0);
        break;

      case 'p':
        tcp_port = strtoul(optarg, NULL, 0);
        tcp_port_is_default_ = false;
        break;

      case 'I':
        status_interval_ms = strtoul(optarg, NULL, 0);
        break;

      case 'o':
        record_inno_pc_filename = optarg;
        break;

      case 'm':
        record_inno_pc_size_in_m = strtoul(optarg, NULL, 0);
        break;

      case 't':
        png_filename = optarg;
        break;

      case 'b':
        rosbag_filename = optarg;
        break;

      case 'B':
        rosbag_size_in_m = strtoul(optarg, NULL, 0);
        break;

      case 'r':
        record_raw_filename = optarg;
        break;

      case 'R':
        record_raw_size_in_m = strtoul(optarg, NULL, 0);
        break;

      case 'g':
        config_filename = optarg;
        break;

      case 'G':
        config_filename2 = optarg;
        break;

      case 'H':
        dtc_filename = optarg;
        break;

      case 'x':
        lidar.use_xyz = strtoul(optarg, NULL, 0);
        break;

      case 'D':
        debug_level = (enum InnoLogLevel)atoi(optarg);
        break;

      case 'N':
        log_filename = optarg;
        break;

      case 'a':
        log_file_rotate_number = strtoul(optarg, NULL, 0);
        break;

      case 'A':
        log_file_max_size_k = strtoul(optarg, NULL, 0);
        break;

      case 'C':
        error_log_filename = optarg;
        break;

      case 'j':
        error_log_file_rotate_number = strtoul(optarg, NULL, 0);
        break;

      case 'J':
        error_log_file_max_size_k = strtoul(optarg, NULL, 0);
        break;

      case 'v':
        get_version = true;
        break;

      case 'V':
        show_viewer = 1;
        break;

      case 'q':
        quiet = 1;
        break;

      case 'E':
        sleep_second = atof(optarg);
        break;

      case 'Y':
        inno_pc_record_npy = true;
        break;

      case 'K':
        run_time_s = optarg;
        break;

      case 'L':
        test_command = optarg;
        break;

      case 'X':
        test_command_send_interval_ms = optarg;
        break;

      case '?':
        abort();

      default:
        inno_log_error("unknown options %c\n", c);
        usage_();
        exit(1);
    }
  }

  if (get_version) {
    exit(0);
  }

  if ((!lidar.processed) &&
      InnoUtils::ends_with(argv[0], "client")) {
    inno_log_info("force processed mode");
    lidar.processed = 1;
  }

  if (lidar.check() != 0) {
    usage_();
    exit(1);
  }

  if (is_client_mode()) {
    if (tcp_port_is_default_) {
      tcp_port = kDefaultClientTcpPort_;
    }

    if (lidar.lidar_port_is_default_) {
      lidar.lidar_port = lidar.kDefaultClientLidarPort_;
    }
  }
}

}  // namespace innovusion

