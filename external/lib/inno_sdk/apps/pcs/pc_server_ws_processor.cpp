/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/pc_server_ws_processor.h"

#include <map>
#include <memory>

#include "pcs/pcs.h"
#include "src/utils/inno_lidar_log.h"
#include "src/ws_utils/server_ws.h"

namespace innovusion {

static std::shared_ptr<WsConnection> get_ws_connection(
    void *conn) {
  return *(std::shared_ptr<WsConnection>*)(conn);
}

PcServerWsProcessor::PcServerWsProcessor(uint16_t port,
                                         uint32_t id, PCS *pcs)
    : ServerWsProcessor(port) {
  lidar_id_ = id;
  pcs_ = pcs;
  inno_log_verify(pcs_, "pcs");

  add_endpoint("^/stream/?.*$", true, handle_stream_s_, this);
  add_endpoint("^/capture/?.*$", false, handle_capture_s_, this);
  add_endpoint("^/command/?.*$", false, handle_command_s_, this);
  add_endpoint("^/start/?.*$", false, handle_start_s_, this);
  add_endpoint("^/download/?.*$", false, handle_download_s_, this);
}

PcServerWsProcessor::~PcServerWsProcessor() {
}

/********************************
 * process query from websocket server
 ********************************/
int PcServerWsProcessor::parse_query_string_(void *conn,
                                             const std::string &nv,
                                             std::string *reply) {
  std::string name, value;
  split_query_string_s_(nv, &name, &value);
  int ret = 0;

  if (name == "subscribe_topics") {
    parse_subscribe_topics_(
        &get_topics_subscribed_s(conn), value);
  } else if (name.find("get_") == 0) {
    ret = pcs_->get_pcs(name.substr(4), value, reply, conn, true);
  } else if (name.find("set_") == 0) {
    ret = pcs_->set_pcs(name.substr(4), value, conn);
  } else if (name == "lidar_id") {
    // do nothing
  } else if (name.size() == 0) {
  } else {
    inno_log_error("Unknown query %s", nv.c_str());
    ret = -1;
  }
  return ret;
}

bool PcServerWsProcessor::verify_id_from_query_(void *conn,
                                                const std::string &nv) {
  size_t epos = nv.find("=");
  if (epos != std::string::npos) {
    std::string name = nv.substr(0, epos);
    std::string value = nv.substr(epos + 1);
    if (name == "lidar_id") {
      uint32_t lidar_id;
      if (sscanf(value.c_str(), "%u", &lidar_id) != 1) {
        // reject if id is not valid
        inno_log_warning("Invalid query %s %s", nv.c_str(), value.c_str());
        return false;
      }
      if (lidar_id != lidar_id_) {
        // reject if id does not match
        inno_log_warning("lidar_id not match %u vs %u",
                         lidar_id, lidar_id_);
        return false;
      }
    }
  }
  return true;
}

int PcServerWsProcessor::parse_and_exec_download_(void *conn) {
  std::string query = get_query_string_s(conn);
  std::vector<std::string> params = InnoUtils::split(query, "&");
  std::map<std::string, std::string> mp;
  for (auto param : params) {
    std::vector<std::string> split = InnoUtils::split(param, "=");
    inno_log_verify(split.size() == 2, "invalid parameter: %s", query.c_str());
    mp[split[0]] = split[1];
  }
  std::string item = mp["item"];
  std::string path = mp["path"];
  std::string offset = mp["offset"];
  std::string length = mp["length"];
  inno_log_debug("type=%s path=%s offset=%s length=%s",
                 item.c_str(), path.c_str(), offset.c_str(), length.c_str());
  int ret = pcs_->send_file(item, path, offset, length, conn);
  return ret;
}

int PcServerWsProcessor::parse_and_exec_capture_(void *conn) {
  std::string query = get_query_string_s(conn);
  std::string delimiter = "&";
  size_t pos = 0;
  std::string token;
  std::string name, value;
  std::map<std::string, std::string> mp;

  while ((pos = query.find(delimiter)) != std::string::npos) {
    token = query.substr(0, pos);
    split_query_string_s_(token, &name, &value);
    mp[name] = value;
    query.erase(0, pos + delimiter.length());
  }
  split_query_string_s_(query, &name, &value);
  mp[name] = value;

  std::string type = mp["type"];
  std::string duration = mp["duration"];
  int ret = pcs_->add_capture_job(type, duration, conn);
  return ret;
}

int PcServerWsProcessor::parse_and_exec_command_(void *conn,
                                                 std::string *reply) {
  std::string delimiter = "&";
  size_t pos = 0;
  std::string token;
  int ret = 0;
  for (int i = 0; i < 2; i++) {
    std::string query = get_query_string_s(conn);
    while ((pos = query.find(delimiter)) != std::string::npos) {
      token = query.substr(0, pos);
      if (i == 0) {
        if (!verify_id_from_query_(conn, token)) {
          return -1;
        }
      } else {
        int r = parse_query_string_(conn, token, reply);
        if (r) {
          ret = r;
        }
      }
      query.erase(0, pos + delimiter.length());
    }
    if (i == 0) {
      if (!verify_id_from_query_(conn, query)) {
        return -1;
      }
    } else {
      int r = parse_query_string_(conn, query, reply);
      if (r) {
        ret = r;
      }
    }
  }
  return ret;
}

/********************************
 * process callbacks from websocket server
 ********************************/
void PcServerWsProcessor::handle_stream_(void *conn) {
  std::string reply;
  int ret = verify_http_crc32_(conn);
  if (ret < 0) {
     return;
  }
  ret = parse_and_exec_command_(conn, &reply);
  if (ret == 0) {
    inno_log_info("add streaming connection");
    add_stream_connection(conn);
    write_ws_socket_preamble_s(conn);
  } else {
    inno_log_warning("parse_and_exec_command return %d", ret);
  }
}

void PcServerWsProcessor::handle_command_(void *conn) {
  char buf[8192];
  std::string reply;
  int written = 0;

  int res = verify_http_crc32_(conn);
  if (res < 0) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_400_template,
                                      strlen(bad_reply_400_template));
    flush_buffer_s(conn);
    return;
  }

  int ret = parse_and_exec_command_(conn, &reply);
  if (ret) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_400_template,
                                     strlen(bad_reply_400_template));
  } else {
    if (res == 0) {  // crc32 is right
      uint32_t crc32 = InnoPacketReader::calculate_http_crc32
                        (reply.c_str(), reply.size());
      written = snprintf(buf, sizeof(buf),
                        reply_200_CRC32_template, reply.size(), crc32);
    } else {
      written = snprintf(buf, sizeof(buf),
                          reply_200_template, reply.size());
    }
    if (written >= static_cast<int>(sizeof(buf))) {
      inno_log_error("run out of buffer %d", written);
      write_buffer_to_ws_socket_full_s(conn, bad_reply_500_template,
                                     strlen(bad_reply_500_template));
    } else {
      write_buffer_to_ws_socket_full_s(conn, buf, written);
      if (reply.size()) {
        write_buffer_to_ws_socket_full_s(conn, reply.c_str(), reply.size());
      }
    }
  }
  flush_buffer_s(conn);
}

void PcServerWsProcessor::handle_capture_(void *conn) {
  int ret = verify_http_crc32_(conn);
  if (ret < 0) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_400_template,
                                      strlen(bad_reply_400_template));
    flush_buffer_s(conn);
    return;
  }

  ret = parse_and_exec_capture_(conn);
  if (ret == 0) {
    // postpone send response
  } else if (ret == 503) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_503_template,
                                     strlen(bad_reply_503_template));
  } else if (ret == 400) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_400_template,
                                     strlen(bad_reply_400_template));
  } else {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_500_template,
                                     strlen(bad_reply_500_template));
  }
  flush_buffer_s(conn);
}

void PcServerWsProcessor::handle_start_(void *conn) {
  std::string reply;
  int ret = verify_http_crc32_(conn);
  if (ret < 0) {
     return;
  }

  ret = parse_and_exec_command_(conn, &reply);
  if (ret == 0) {
    inno_log_info("add streaming connection");
    add_stream_connection(conn);
  }
}

void PcServerWsProcessor::handle_download_(void *conn) {
  int ret = verify_http_crc32_(conn);
  if (ret < 0) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_400_template,
                                     strlen(bad_reply_400_template));
    flush_buffer_s(conn);
    return;
  }

  ret = parse_and_exec_download_(conn);
  if (ret == 0) {
    // postpone send response
  } else if (ret == 503) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_503_template,
                                     strlen(bad_reply_503_template));
  } else if (ret == 400) {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_400_template,
                                     strlen(bad_reply_400_template));
  } else {
    write_buffer_to_ws_socket_full_s(conn, bad_reply_500_template,
                                     strlen(bad_reply_500_template));
  }
  flush_buffer_s(conn);
}

int PcServerWsProcessor::write_ws_socket_cframe_s(
      void *conn, uint32_t context,
      const struct ::inno_cframe_header *cframe) {
  size_t payload_size = cframe->get_size();
  if (payload_size <= 0) {
    return 0;
  }
  return ServerWs::write_ws_socket_var_struct_s(
      get_ws_connection(conn),
      context,
      cframe,
      sizeof(*cframe),
      payload_size);
}

int PcServerWsProcessor::write_ws_socket_cpacket_s(
    void *conn, uint32_t context,
    const InnoCommonHeader *packet) {
  int ret = write_buffer_to_ws_socket_full_s(conn,
                                             (const char*)packet,
                                             packet->size);
  if (ret >= 0) {
    int ret2 = flush_buffer_s(conn);
    if (ret2 < 0) {
      return ret2;
    }
  }
  return ret;
}

int PcServerWsProcessor::write_ws_socket_cpacket(
    uint32_t context,
    const InnoCommonHeader *packet) {
  if (pc_server_) {
    return pc_server_->write_socket_var_struct(
                       packet,
                       packet->size);
  }
  return 0;
}

int PcServerWsProcessor::write_ws_socket_cframe(
    uint32_t context,
    const struct ::inno_cframe_header *cframe) {
  ssize_t payload_size = cframe->get_size();
  if (payload_size <= 0) {
    return 0;
  }
  int ret = pc_server_->write_ws_socket_var_struct(
      context,
      cframe,
      sizeof(*cframe),
      payload_size);
  return ret;
}

int PcServerWsProcessor::verify_http_crc32_(void *conn) {
  std::string recv_buf, recv_url;
  recv_buf = PcServerWsProcessor::get_recv_buffer_s(conn);
  recv_url = PcServerWsProcessor::get_url_s(conn);
  return InnoPacketReader::verify_http_crc32(recv_buf.c_str(),
                                             recv_url.c_str());
}

}  // namespace innovusion

