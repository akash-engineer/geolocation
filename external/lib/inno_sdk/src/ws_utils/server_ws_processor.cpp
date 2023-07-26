/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "src/ws_utils/server_ws_processor.h"

#include <map>
#include <memory>

#include "src/utils/inno_lidar_log.h"
#include "src/ws_utils/server_ws.h"

namespace innovusion {

const char *ServerWsProcessor::reply_capture_template =
    "HTTP/1.1 200 OK\r\n"
    "Server: Innovusion PCS\r\n"
    "Cache-Control: no-cache\r\n"
    "Content-Type: %s\r\n"
    "Transfer-Encoding: chunked\r\n"
    "Content-Disposition: attachment; filename=\"%s\"\r\n"
    "Connection: close\r\n\r\n";

const char *ServerWsProcessor::reply_200_template =
    "HTTP/1.1 200 OK\r\n"
    "Server: Innovusion PCS\r\n"
    "Cache-Control: no-cache\r\n"
    "Content-Length: %u\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n\r\n";

const char *ServerWsProcessor::reply_200_CRC32_template =
    "HTTP/1.1 200 OK\r\n"
    "Server: Innovusion PCS\r\n"
    "Cache-Control: no-cache\r\n"
    "Content-Length: %u\r\n"
    "Content-Type: text/html\r\n"
    "X-INNO-CRC32: %8x\r\n"
    "Connection: close\r\n\r\n";

const char *ServerWsProcessor::bad_reply_400_template =
    "HTTP/1.1 400 Bad Request\r\n"
    "Server: Innovusion PCS\r\n"
    "Content-Length: 0\r\n"
    "Connection: close\r\n\r\n";

const char *ServerWsProcessor::bad_reply_500_template =
    "HTTP/1.1 500 Internal Server Error\r\n"
    "Server: Innovusion PCS\r\n"
    "Content-Length: 0\r\n"
    "Connection: close\r\n\r\n";

const char *ServerWsProcessor::bad_reply_503_template =
    "HTTP/1.1 503 Service Unavailable\r\n"
    "Server: Innovusion PCS\r\n"
    "Content-Length: 0\r\n"
    "Connection: close\r\n\r\n";

static std::shared_ptr<WsConnection> get_ws_connection(
    void *conn) {
  return *(std::shared_ptr<WsConnection>*)(conn);
}

ServerWsProcessor::ServerWsProcessor(uint16_t port) {
  pc_server_ = new ServerWs(port);
  inno_log_verify(pc_server_, "pc_server");
}

ServerWsProcessor::~ServerWsProcessor() {
  delete pc_server_;
  pc_server_ = NULL;
}

void ServerWsProcessor::parse_subscribe_topic_(std::vector<int> *sub,
                                               const std::string &topic) {
  try {
    int v = std::stoi(topic);
    inno_log_info("subscribe topic %d\n", v);
    sub->push_back(v);
  }
  catch(...) {
  }
}

void ServerWsProcessor::parse_subscribe_topics_(std::vector<int> *sub,
                                                const std::string &value) {
  std::string delimiter = ",";
  size_t pos = 0;
  std::string token;
  std::string topics = value;
  while ((pos = topics.find(delimiter)) != std::string::npos) {
    token = topics.substr(0, pos);
    parse_subscribe_topic_(sub, token);
    topics.erase(0, pos + delimiter.length());
  }
  parse_subscribe_topic_(sub, topics);
}

void ServerWsProcessor::split_query_string_s_(const std::string &nv,
                                              std::string *name,
                                              std::string *value) {
  size_t epos = nv.find("=");
  if (epos != std::string::npos) {
    *name = nv.substr(0, epos);
    *value = nv.substr(epos + 1);
  } else {
    *name = nv;
    *value = "";
  }
  return;
}

std::string &ServerWsProcessor::get_query_string_s(void *conn) {
  return get_ws_connection(conn)->query_string;
}

std::vector<int> &ServerWsProcessor::get_topics_subscribed_s(void *conn) {
  return get_ws_connection(conn)->topics_subscribed;
}

std::string ServerWsProcessor::get_source_ip_string_s(void *conn) {
  return get_ws_connection(conn)->get_source_ip_string();
}

std::string ServerWsProcessor::get_recv_buffer_s(void *conn) {
  return get_ws_connection(conn)->get_recv_buffer();
}

std::string ServerWsProcessor::get_url_s(void *conn) {
  return get_ws_connection(conn)->get_url();
}

int ServerWsProcessor::flush_buffer_s(void *conn) {
  return get_ws_connection(conn)->flush_buffer();
}

void ServerWsProcessor::close_connection_s(void *conn) {
  flush_buffer_s(conn);
}

int ServerWsProcessor::write_buffer_to_ws_socket_full_s(
    void *conn, const char *buf, ssize_t input_len) {
  return ServerWs::write_buffer_to_ws_socket_full_s(
      get_ws_connection(conn),
      buf, input_len);
}


int ServerWsProcessor::write_chunk_to_ws_socket_full_s(
    void *conn, const char *buf, ssize_t input_len) {
  int total = 0;
  char chunk[100];

  snprintf(chunk, sizeof(chunk), "%lx\r\n", input_len);
  int ret = ServerWs::write_buffer_to_ws_socket_full_s(
      get_ws_connection(conn),
      chunk, strlen(chunk));
  if (ret >= 0) {
    total += strlen(chunk);
  } else {
    return ret;
  }

  if (input_len && buf) {
    ret = ServerWs::write_buffer_to_ws_socket_full_s(
        get_ws_connection(conn),
        buf, input_len);
    if (ret >= 0) {
      total += input_len;
    } else {
      return ret;
    }
  }

  ret = ServerWs::write_buffer_to_ws_socket_full_s(
      get_ws_connection(conn),
      "\r\n", 2);
  if (ret >= 0) {
    total += 2;
    ret = flush_buffer_s(conn);
  } else {
    return ret;
  }
  if (ret >= 0) {
    return total;
  } else {
    return ret;
  }
}

int ServerWsProcessor::write_buffer_to_ws_socket_with_length_s(
    void *conn, const char *buf, ssize_t input_len) {
  return ServerWs::write_buffer_to_ws_socket_with_length_s(
      get_ws_connection(conn), buf, input_len);
}

int ServerWsProcessor::write_ws_socket_preamble_s(
    void *conn) {
  return ServerWs::write_ws_socket_preamble_s(
      get_ws_connection(conn));
}

bool ServerWsProcessor::has_ws_socket() {
  return pc_server_->has_ws_socket();
}

void ServerWsProcessor::add_endpoint(const char *path,
                                     bool is_ws,
                                     WsServerProcessorCallback callback,
                                     void *ctx) {
  return pc_server_->add_endpoint(path, is_ws, callback, ctx);
}

void ServerWsProcessor::add_stream_connection(
    void *conn) {
  return pc_server_->add_stream_connection(
      get_ws_connection(conn));
}

void ServerWsProcessor::start() {
  return pc_server_->start();
}

void ServerWsProcessor::stop() {
  return pc_server_->stop();
}

}  // namespace innovusion
