  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef PCS_UDP_SENDER_H_
#define PCS_UDP_SENDER_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <string>

namespace innovusion {
class UdpSender {
 public:
  explicit UdpSender(const std::string &ip, uint16_t port);
  ~UdpSender();

  static constexpr uint16_t kUdpMaxMsgSize =
      65535 - 20 /*ip header*/ - 8 /*udp header*/;

 public:
  ssize_t write(const void *buffer, size_t size, bool blocking = true);

  ssize_t write_raw(const void *header, size_t header_size, const void *body,
                    size_t body_size);

  std::string get_udp_ip_string() const {
    return udp_ip_str_;
  }

  bool is_multicast() const {
    return multicast_;
  }

  std::string get_ip_str() const {
    return udp_ip_str_;
  }

  uint16_t get_port() const {
    return htons(sockaddr_.sin_port);
  }

 private:
  std::string udp_ip_str_;
  struct sockaddr_in sockaddr_;
  int fd_;
  bool multicast_;
};

}  // namespace innovusion

#endif  // PCS_UDP_SENDER_H_
