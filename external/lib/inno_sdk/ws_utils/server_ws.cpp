/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "ws_utils/server_ws.h"

#include <getopt.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>   // NOLINT
#include <string>

#include "utils/inno_lidar_log.h"

namespace innovusion {

ServerWs::ServerWs(uint16_t port)
    : server_(new WsServer(this))
    , server_thread_(NULL) {
  setup_server_(port);
  total_bytes_ = 0;
  total_called_ = 0;
  last_show_bytes_ = 0;
  last_show_called_ = 0;
  clock_gettime(CLOCK_MONOTONIC_RAW, &last_show_call_spec_);
}

ServerWs::~ServerWs() {
  if (server_thread_) {
    stop();
  }
}

ServerWs *ServerWs::get_inst_(std::shared_ptr<WsServer::Connection> conn) {
  return reinterpret_cast<ServerWs *>(conn->ctx);
}

void ServerWs::on_error_(std::shared_ptr<WsServer::Connection> conn,
                         const char* ec) {
  inno_log_info("Server: Event in connection %p. message: %s",
                conn.get(), ec);
  get_inst_(conn)->remove_stream_connection_(conn);
}

void ServerWs::on_send_timeout_(std::shared_ptr<WsServer::Connection> conn,
                                int timeout, const char* ec_msg) {
  inno_log_info("Server: send timeout in %ds, connection %p. message: %s",
               timeout, conn.get(), ec_msg);
  get_inst_(conn)->remove_stream_connection_(conn);
}

void ServerWs::on_close_(std::shared_ptr<WsServer::Connection> conn,
                         int status,
                         const std::string & /*reason*/) {
  inno_log_info("Server: Close connection %p. status: %d",
                conn.get(), status);
  get_inst_(conn)->remove_stream_connection_(conn);
}

void ServerWs::on_open_(std::shared_ptr<WsConnection> conn,
                        const std::string &path) {
  inno_log_info("Server: Connect connection %p. path: %s",
                conn.get(), path.c_str());
  get_inst_(conn)->on_conn_open_(conn, path);
}

void ServerWs::setup_ws_endpoint_(const char *path,
                                  bool is_sp) {
  auto& ws_endpoint = server_->endpoint[path];
  ws_endpoint.set_sp(is_sp);
  ws_endpoint.on_open = [this](std::shared_ptr<WsConnection> conn,
                               const std::string &path) {
                          on_conn_open_(conn, path);
                        };
  ws_endpoint.on_close = on_close_;
  ws_endpoint.on_error = on_error_;
  ws_endpoint.on_send_timeout = on_send_timeout_;
}

void ServerWs::setup_server_(uint16_t port) {
  server_->config.port = port;
}

void ServerWs::remove_stream_connection_(std::shared_ptr<WsConnection> conn) {
  std::unique_lock<std::mutex> lk(mutex_);
  stream_connections_.remove(conn);
}

void ServerWs::on_conn_open_(std::shared_ptr<WsConnection> conn,
                               const std::string &path) {
  inno_log_info("Server: Connect connection %p. "
                "path: %s query: %s. From %s",
                conn.get(), path.c_str(), conn->query_string.c_str(),
                conn->get_source_ip_string().c_str());
  PcServerCallback callback = callback_[path];
  inno_log_verify(callback, "no callbck for path %s", path.c_str());
  (*callback)(callback_ctx_[path], &conn);
}

void ServerWs::bw_stats_(int total) {
  struct timespec this_call;
  clock_gettime(CLOCK_MONOTONIC_RAW, &this_call);

  total_called_++;
  total_bytes_ += total;
  if (total_bytes_ - last_show_bytes_ > 100 * 1024 * 1024) {
    uint64_t diff_us = 1000000;
    if (last_show_bytes_ > 0) {
      diff_us = (this_call.tv_sec - last_show_call_spec_.tv_sec) * 1000000UL +
                (this_call.tv_nsec - last_show_call_spec_.tv_nsec) / 1000;
    }
    if (diff_us > 0) {
      size_t diff_byte = total_bytes_ - last_show_bytes_;
      size_t diff_called = total_called_ - last_show_called_;
      // inno_log_info may trigger message callback
      // to avoid infinity loop, have to set last_show_call_ and
      // last_show_bytes_ here
      last_show_call_spec_ = this_call;
      last_show_bytes_ = total_bytes_;
      last_show_called_ = total_called_;
      inno_log_info("Total clients: %lu, bytes served: %lu, called: %lu, "
                    "%.2f MB/s\n",
                    stream_connections_.size(),
                    diff_byte, diff_called,
                    diff_byte * 1.0 / diff_us);
    }
  }
  return;
}

int ServerWs::write_buffer_to_ws_socket_full_s(
    std::shared_ptr<WsConnection> conn,
    const char *buf,
    ssize_t input_len) {
  return conn->write_to_buffer(buf, input_len);
}

int ServerWs::write_buffer_to_ws_socket_with_length_s(
    std::shared_ptr<WsServer::Connection> conn, const char *buf,
    ssize_t input_len) {
  uint32_t len = static_cast<uint32_t>(htonl(input_len));
  int ret = write_buffer_to_ws_socket_full_s(conn, (const char*)&len,
                                             sizeof(len));
  int total = 0;
  if (ret >= 0) {
    total += ret;
    ret = write_buffer_to_ws_socket_full_s(conn, buf, input_len);
  }
  if (ret >= 0) {
    total += ret;
    return total;
  } else {
    return ret;
  }
}

int ServerWs::write_ws_socket_var_struct_s(
    std::shared_ptr<WsConnection> conn,
    uint32_t context,
    const void *var_struct,
    size_t struct_size,
    size_t additional_size) {
  if (var_struct == NULL) {
    return 0;
  }
  int h = htonl(context);
  int ret, total = 0;
  ret = write_buffer_to_ws_socket_with_length_s(conn,
                                                (const char *)&h,
                                                sizeof(h));
  if (ret >= 0) {
    total += ret;
    ret = write_buffer_to_ws_socket_with_length_s(conn,
                                                  (const char *)var_struct,
                                                  struct_size);
  } else {
    return ret;
  }
  if (ret >= 0) {
    total += ret;
    ret = write_buffer_to_ws_socket_with_length_s(
        conn,
        reinterpret_cast<const char *>(var_struct) +
        struct_size,
        additional_size);
  } else {
    return ret;
  }

  if (ret >= 0) {
    total += ret;
    ret = conn->flush_buffer();
  } else {
    return ret;
  }

  if (ret >= 0) {
    return total;
  } else {
    return ret;
  }
}

int ServerWs::write_ws_socket_preamble_s(std::shared_ptr<WsConnection> conn) {
  const char *vstr = "PS32";
  int ret = write_buffer_to_ws_socket_full_s(conn, vstr, strlen(vstr));
  if (ret >= 0) {
    int ret2 = conn->flush_buffer();
    if (ret2 < 0) {
      return ret2;
    }
  }
  return ret;
}

int ServerWs::write_socket_var_struct(
    const void *var_struct,
    size_t total_size) {
  int total = 0;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    for (auto conn : stream_connections_) {
      if (conn->is_sp_conn()) {
        int ret = write_buffer_to_ws_socket_full_s(
            conn,
            (const char*)var_struct,
            total_size);
        if (ret >= 0) {
          total += ret;
          conn->flush_buffer();
        }
      }
    }
  }
  if (total > 0) {
    // have to do it without lock
    bw_stats_(total);
  }
  return 0;
}

int ServerWs::write_ws_socket_var_struct(
    uint32_t context,
    const void *var_struct,
    size_t struct_size,
    size_t additional_size) {
  int total = 0;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    for (auto conn : stream_connections_) {
      if (!conn->is_sp_conn()) {
        int ret = write_ws_socket_var_struct_s(conn,
                                               context,
                                               var_struct,
                                               struct_size,
                                               additional_size);
        if (ret >= 0) {
          total += ret;
        }
      }
    }
  }
  if (total > 0) {
    // have to do it without lock
    bw_stats_(total);
  }
  return 0;
}

bool ServerWs::has_ws_socket() {
  std::unique_lock<std::mutex> lk(mutex_);
  for (auto conn : stream_connections_) {
    if (!conn->is_sp_conn()) {
      return true;
    }
  }
  return false;
}

void ServerWs::add_endpoint(const char *path,
                            bool is_ws,
                            PcServerCallback callback,
                            void *ctx) {
  setup_ws_endpoint_(path, !is_ws);
  inno_log_verify(callback_[path] == NULL,
                  "already has callbck for path %s",
                  path);
  callback_[path] = callback;
  callback_ctx_[path] = ctx;
}

void ServerWs::add_stream_connection(std::shared_ptr<WsConnection> conn) {
  std::unique_lock<std::mutex> lk(mutex_);
  stream_connections_.push_back(conn);
}

void ServerWs::start() {
  inno_log_info("Server listen on port %hu", server_->config.port);
  inno_log_verify(server_thread_ == NULL,
                  "server_thread should be NULL");
  server_thread_ = new std::thread([this]() {
                                     server_->start();
                                   });
  inno_log_verify(server_thread_, "server_thread");
}

void ServerWs::stop() {
  server_->stop();
  inno_log_verify(server_thread_, "server_thread");
  server_thread_->join();
  delete server_thread_;
  server_thread_ = NULL;
  return;
}

}  // namespace innovusion
