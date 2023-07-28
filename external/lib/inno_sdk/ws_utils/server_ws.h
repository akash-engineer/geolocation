/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef WS_UTILS_SERVER_WS_H_
#define WS_UTILS_SERVER_WS_H_

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>   // NOLINT
#include <string>

#include "ws_utils/server_ws/server_ws.hpp"

using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;
using WsConnection = WsServer::Connection;

namespace innovusion {

typedef void (*PcServerCallback)(void *ctx,
                                 void *conn);

class ServerWs {
 public:
  explicit ServerWs(uint16_t port);
  ~ServerWs();

 public:
  static int write_buffer_to_ws_socket_full_s(
      std::shared_ptr<WsConnection> conn,
      const char *buf,
      ssize_t input_len);
  static int write_buffer_to_ws_socket_with_length_s(
      std::shared_ptr<WsServer::Connection> conn, const char *buf,
      ssize_t input_len);
  static int write_ws_socket_var_struct_s(
      std::shared_ptr<WsConnection> conn,
      uint32_t context,
      const void *var_struct,
      size_t struct_size,
      size_t additional_size);
  static int write_ws_socket_preamble_s(std::shared_ptr<WsConnection> conn);

 private:
  static ServerWs *get_inst_(std::shared_ptr<WsServer::Connection> conn);
  static void on_error_(std::shared_ptr<WsServer::Connection> conn,
                        const char*);
  static void on_close_(std::shared_ptr<WsServer::Connection> conn,
                        int status,
                        const std::string & /*reason*/);
  static void on_open_(std::shared_ptr<WsConnection> conn,
                       const std::string &path);
  static void on_send_timeout_(std::shared_ptr<WsServer::Connection> conn,
                               int timeout, const char* ec_msg);

 private:
  void setup_ws_endpoint_(const char *path, bool is_sp);
  void setup_server_(uint16_t port);
  void remove_stream_connection_(std::shared_ptr<WsConnection> conn);
  void on_conn_open_(std::shared_ptr<WsConnection> conn,
                     const std::string &path);
  void bw_stats_(int total);

 public:
  int write_socket_var_struct(
      const void *var_struct,
      size_t total_size);

  int write_ws_socket_var_struct(
      uint32_t context,
      const void *var_struct,
      size_t struct_size,
      size_t additional_size);

  bool has_ws_socket();

 public:
  void add_endpoint(const char *path,
                    bool is_ws,
                    PcServerCallback callback,
                    void *ctx);
  void add_stream_connection(std::shared_ptr<WsConnection> conn);
  void start();
  void stop();

 private:
  std::shared_ptr<WsServer> server_;
  std::thread *server_thread_;
  std::list<std::shared_ptr<WsConnection>> stream_connections_;
  std::map<std::string, PcServerCallback> callback_;
  std::map<std::string, void *> callback_ctx_;
  std::mutex mutex_;

  uint64_t total_called_;
  uint64_t total_bytes_;
  uint64_t last_show_bytes_;
  uint64_t last_show_called_;
  struct timespec last_show_call_spec_;
};

}  // namespace innovusion

#endif  // WS_UTILS_SERVER_WS_H_
