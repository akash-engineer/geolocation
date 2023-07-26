/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_PC_SERVER_WS_PROCESSOR_H_
#define PCS_PC_SERVER_WS_PROCESSOR_H_

#include <getopt.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include "src/sdk_common/converter/cframe_legacy.h"
#include "src/sdk_common/inno_lidar_packet.h"
#include "src/ws_utils/server_ws_processor.h"

namespace innovusion {
class PCS;
class ServerWs;

typedef void (*PcServerWrapperCallback)(void *ctx,
                                        void *conn);

class PcServerWsProcessor : public ServerWsProcessor {
 public:
  explicit PcServerWsProcessor(uint16_t port, uint32_t id, PCS *pcs);
  ~PcServerWsProcessor();

 private:
  static void handle_stream_s_(void *ctx, void *conn) {
    (reinterpret_cast<PcServerWsProcessor *>(ctx))->handle_stream_(conn);
  }
  static void handle_command_s_(void *ctx, void *conn) {
    (reinterpret_cast<PcServerWsProcessor *>(ctx))->handle_command_(conn);
  }
  static void handle_capture_s_(void *ctx, void *conn) {
    (reinterpret_cast<PcServerWsProcessor *>(ctx))->handle_capture_(conn);
  }
  static void handle_start_s_(void *ctx, void *conn) {
    (reinterpret_cast<PcServerWsProcessor *>(ctx))->handle_start_(conn);
  }
  static void handle_download_s_(void *ctx, void *conn) {
    (reinterpret_cast<PcServerWsProcessor *>(ctx))->handle_download_(conn);
  }

 public:
  static int write_ws_socket_cframe_s(
      void *conn, uint32_t context,
      const struct ::inno_cframe_header *cframe);
  static int write_ws_socket_cpacket_s(
      void *conn, uint32_t context,
      const InnoCommonHeader *packet);

 private:
  void handle_stream_(void *conn);
  void handle_command_(void *conn);
  void handle_capture_(void *conn);
  void handle_start_(void *conn);
  void handle_download_(void *conn);

 private:
  int parse_query_string_(void *conn, const std::string &nv,
                          std::string *reply);
  bool verify_id_from_query_(void *conn, const std::string &nv);
  int parse_and_exec_command_(void *conn, std::string *reply);
  int parse_and_exec_capture_(void *conn);
  int parse_and_exec_download_(void *conn);
  int verify_http_crc32_(void *conn);

 public:
  int write_ws_socket_cpacket(uint32_t context, const InnoCommonHeader *packet);
  int write_ws_socket_cframe(uint32_t context,
                             const struct ::inno_cframe_header *cframe);

 private:
  uint32_t lidar_id_;
  PCS *pcs_;
};

}  // namespace innovusion

#endif  // PCS_PC_SERVER_WS_PROCESSOR_H_
