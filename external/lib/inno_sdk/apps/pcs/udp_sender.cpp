  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/udp_sender.h"

#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <string>

#include "src/utils/inno_lidar_log.h"

namespace innovusion {
UdpSender::UdpSender(const std::string &ip, uint16_t port) {
  memset(reinterpret_cast<char *>(&sockaddr_), 0,
         sizeof(sockaddr_));
  multicast_ = false;
  fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd_ < 0) {
    inno_log_error_errno("cannot open udp fd %s", ip.c_str());
    exit(1);
  }

  char host_broadcast[256];
  const char *udp_ip;
  if (ip.size() > 0 &&
      isdigit(static_cast<u_char>(ip.c_str()[0])) == 0) {
    int sock;
    struct ifreq ifreq;
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      inno_log_error_errno("cannot open sock %s", ip.c_str());
      exit(1);
    }
    memset(&ifreq, 0, sizeof ifreq);
    strncpy(ifreq.ifr_name, ip.c_str(), IFNAMSIZ);

    if (ioctl(sock, SIOCGIFBRDADDR, &ifreq) != 0) {
      inno_log_error("Could not find interface named %s", ip.c_str());
      exit(1);
    }
    if (0 != getnameinfo(&ifreq.ifr_broadaddr, sizeof(ifreq.ifr_broadaddr),
                         host_broadcast, sizeof(host_broadcast),
                         0, 0, NI_NUMERICHOST)) {
      inno_log_error_errno("Could getnameinfo %s", ip.c_str());
      exit(1);
    }
    udp_ip = host_broadcast;
    close(sock);
    inno_log_info("Interface %s broadcast ip is %s",
                  ip.c_str(), udp_ip);
  } else {
    udp_ip = ip.c_str();
  }
  if (strlen(udp_ip) > 4 &&
      strcmp(udp_ip + strlen(udp_ip) - 4, ".255") == 0) {
    int broadcastEnable = 1;
    int ret = setsockopt(fd_, SOL_SOCKET, SO_BROADCAST,
                         &broadcastEnable, sizeof(broadcastEnable));
    if (ret) {
      inno_log_error_errno("Could not open set socket to broadcast mode %s",
                           udp_ip);
      exit(1);
    } else {
      inno_log_info("%s is broadcast address, set broadcast socket enable",
                    udp_ip);
    }
  }
  sockaddr_.sin_family = AF_INET;
  sockaddr_.sin_port = htons(port);
  if (inet_aton(udp_ip,
                &sockaddr_.sin_addr) == 0) {
    inno_log_panic("%s is not a valid ip",
                   udp_ip);
  }

  char *ip_str = reinterpret_cast<char *>(&sockaddr_.sin_addr);
  int i = ip_str[0] & 0xFF;
  // we will check only first byte of IP
  // and if it from 224 to 239, then it can
  // represent multicast IP.
  if (i >= 224 && i <= 239) {
    unsigned char ttl = 2;
    int ret = setsockopt(fd_, IPPROTO_IP,
                         IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    if (ret) {
      inno_log_error_errno("Could not open set socket to multicast mode %s",
                           udp_ip);
      exit(1);
    }

    // disable loopback
    unsigned char loop = 0;
    setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

    multicast_ = true;
    inno_log_info("%s is multicast address", udp_ip);
  } else {
    inno_log_info("%s is not multicast address", udp_ip);
  }

  udp_ip_str_ = udp_ip;
  inno_log_info("udp sender setup success ip:port=%s:%hu", udp_ip, port);
}

UdpSender::~UdpSender() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

ssize_t UdpSender::write(const void *buffer, size_t size, bool blocking) {
  int flags = 0;
  if (!blocking) {
    flags |= MSG_DONTWAIT;
  }
  ssize_t written = sendto(fd_, buffer, size, flags,
                           (struct sockaddr *)&sockaddr_,
                           sizeof(sockaddr_));
  return written;
}

//
// udp with mutil msg
//
ssize_t UdpSender::write_raw(const void *header, size_t header_size,
                             const void *body, size_t body_size) {
  ssize_t written = 0;

  {
    ssize_t ret = sendto(fd_, header, header_size, MSG_MORE,
                      (struct sockaddr *)&sockaddr_, sizeof(sockaddr_));
    if (ret == -1) {
      inno_log_warning("sendto faild, errno: %d", errno);
    } else {
      written += ret;
    }
  }

  {
    ssize_t ret = sendto(fd_, body, body_size, 0, (struct sockaddr *)&sockaddr_,
                      sizeof(sockaddr_));
    if (ret == -1) {
      inno_log_warning("sendto faild, errno: %d", errno);
    } else {
      written += ret;
    }
  }

  return written;
}

}  // namespace innovusion
