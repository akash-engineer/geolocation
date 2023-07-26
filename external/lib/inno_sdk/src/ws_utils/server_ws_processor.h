/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef WS_UTILS_SERVER_WS_PROCESSOR_H_
#define WS_UTILS_SERVER_WS_PROCESSOR_H_

#include <getopt.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <string>
#include <vector>

namespace innovusion {
class ServerWs;

typedef void (*WsServerProcessorCallback)(void *ctx,
                                          void *conn);
class ServerWsProcessor {
 public:
  explicit ServerWsProcessor(uint16_t port);
  ~ServerWsProcessor();

 public:
  static void parse_subscribe_topic_(std::vector<int> *sub,
                                     const std::string &topic);
  static void parse_subscribe_topics_(std::vector<int> *sub,
                                      const std::string &value);
  static std::vector<int> &get_topics_subscribed_s(void *conn);
  static std::string &get_query_string_s(void *conn);
  static std::string get_source_ip_string_s(void *conn);
  static std::string get_recv_buffer_s(void *conn);
  static std::string get_url_s(void *conn);
  static int flush_buffer_s(void *conn);
  static void close_connection_s(void *conn);
  static int write_buffer_to_ws_socket_full_s(void *conn,
                                              const char *buf,
                                              ssize_t input_len);
  static int write_chunk_to_ws_socket_full_s(void *conn,
                                             const char *buf,
                                             ssize_t input_len);
  static int write_buffer_to_ws_socket_with_length_s(
      void *conn, const char *buf, ssize_t input_len);
  static int write_ws_socket_preamble_s(void *conn);

  static void split_query_string_s_(const std::string &nv,
                                    std::string *name,
                                    std::string *value);

 public:
  bool has_ws_socket();

 public:
  void add_endpoint(const char *path,
                    bool is_ws,
                    WsServerProcessorCallback callback,
                    void *ctx);
  void add_stream_connection(void *conn);
  void start();
  void stop();

 public:
  static const char *reply_capture_template;
  static const char *reply_200_template;
  static const char *reply_200_CRC32_template;
  static const char *bad_reply_400_template;
  static const char *bad_reply_500_template;
  static const char *bad_reply_503_template;

 protected:
  ServerWs *pc_server_;
};

}  // namespace innovusion

#endif  // WS_UTILS_SERVER_WS_PROCESSOR_H_
