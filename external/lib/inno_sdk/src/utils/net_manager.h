/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_NET_MANAGER_H_
#define UTILS_NET_MANAGER_H_

#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#ifndef __MINGW64__
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#else
#include <ws2tcpip.h>
#endif

#include <sys/stat.h>
#include <sys/types.h>

#if !(defined(_QNX_) || defined(__MINGW64__))
  #include <sys/sendfile.h>
#endif

namespace innovusion {
class NetManager {
 private:
  static const uint16_t kDefaultHttpPort = 80;
  static const uint16_t kDefaultLidarPort = 8002;
  static const double kDefaultReadTimeoutSec;
  static const double kMinReadTimeoutSec;
  static const size_t kDefaultRecvBufferSize = 128 * 1024;

 public:
  static int get_connection(const char *ip,
                            unsigned short port,
                            double read_timeout_sec = kDefaultReadTimeoutSec,
                            int recv_buffer_size = kDefaultRecvBufferSize);
  static int send_command_with_fd(int fd, char *reply, int *reply_len,
                                  const char *cmd_fmt, ...)
      __attribute__((format(printf, 4, 5)));
  static ssize_t recv_full_buffer(int fd, char *buffer,
                                  size_t recv_len, int flag);
  static ssize_t write_full_buffer(int fd, const void *buf, size_t count);

 private:
  static int send_command_with_fd_v_(int fd, char *reply, int *reply_len,
                                     const char *cmd_fmt, va_list valist);

 public:
  NetManager(const char *ip,
             unsigned short port = kDefaultLidarPort,
             double timeout_sec = kDefaultReadTimeoutSec);
  ~NetManager();
  void set_base_port(uint16_t port) {
    port_ = port;
  }
  int get_connection(double timeout_sec,
                     int recv_buffer_size = kDefaultRecvBufferSize);
  int http_get(const char *url, char *buffer, size_t buff_size,
               int *status_code, char **content, int *content_size,
               double timeout_sec, unsigned short port = 0);
  int recv_file(int file_fd, int expect_md5, const char *cmd, ...)
      __attribute__((format(printf, 4, 5)));
  int recv_length_buffer(char *buff, int buff_len, int expect_md5,
                         const char *cmd, ...)
      __attribute__((format(printf, 5, 6)));
  int send_file(const char *filename, const char *cmd, ...)
      __attribute__((format(printf, 3, 4)));
  int send_command_return_fd(char *reply, int *reply_len,
                             const char *cmd_fmt, ...)
      __attribute__((format(printf, 4, 5)));
  char *send_command_and_get_reply(const char *cmd, ...)
      __attribute__((format(printf, 2, 3)));
  int send_command_and_free_reply(const char *cmd, ...)
      __attribute__((format(printf, 2, 3)));
  int send_command_and_print_reply(const char *cmd, ...)
      __attribute__((format(printf, 2, 3)));
  void set_default_timeout_sec(double s) {
    default_timeout_sec_ = s;
  }
  static int inno_inet_pton(const char *src, struct in_addr *dst);

 private:
  char *send_command_and_get_reply_v_(const char *cmd,
                                      va_list valist);
  int send_command_and_print_reply_v_(const char *cmd,
                                      va_list valist);

 protected:
  char ip_[64];

 private:
  uint16_t port_;
  double default_timeout_sec_;

 public:
  static class NetManagerInit {
   public:
    NetManagerInit() {
#ifdef __MINGW64__
      WSAStartup(MAKEWORD(2, 2), &wsaData_);
#endif
    }

#ifdef __MINGW64__

   private:
    WSADATA wsaData_;
#endif
  } manager_init;
};

}  // namespace innovusion
#endif  // UTILS_NET_MANAGER_H_
