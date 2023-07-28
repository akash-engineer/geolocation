/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "utils/utils.h"

#include <fcntl.h>
#include <pthread.h>
#ifndef __MINGW64__
#include <netinet/in.h>
#else
#include <winsock2.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#if defined(__i386__) || defined(__x86_64__)
#ifndef __MINGW64__
#include <x86intrin.h>
#else
#include <intrin.h>
#endif
#endif

#ifdef _QNX_
#define SCHED_IDLE 5
#endif

#include <vector>

#include "utils/log.h"
#include "utils/net_manager.h"

namespace innovusion {

const char *InnoUtils::white_space_ = "\t\n\v\f\r ";
const char* InnoUtils::kInnoSeparator = "\a\a";

void InnoUtils::set_self_thread_priority(int priority) {
#ifndef __MINGW64__
  struct sched_param params;
  struct sched_param current_params;
  int policy;
  int current_policy;
  pthread_t this_thread = pthread_self();

  int ret = pthread_getschedparam(this_thread, &current_policy,
                                  &current_params);
  if (ret) {
    inno_log_error_errno("getschedparam %d", ret);
    return;
  } else {
    inno_log_trace("thread current priority is %d (%d), target is %d",
                   current_params.sched_priority, current_policy,
                   priority);
  }
  if (priority == 0) {
    return;
  } else if (priority > 0) {
    policy = SCHED_FIFO;
    params.sched_priority = current_params.sched_priority + priority;
  } else {
    policy = SCHED_IDLE;
    params.sched_priority = 0;
  }
  if (params.sched_priority > 99) {
    params.sched_priority = 99;
  }
  if (params.sched_priority < 0) {
    params.sched_priority = 0;
  }
  ret = pthread_setschedparam(this_thread, policy, &params);
  if (ret != 0) {
    inno_log_warning_errno("setschedparam(%d)", params.sched_priority);
    return;
  }
  ret = pthread_getschedparam(this_thread, &current_policy,
                              &current_params);
  if (ret) {
    inno_log_error_errno("getschedparam 2 %d", ret);
  } else {
    if (current_params.sched_priority != params.sched_priority) {
      inno_log_error("current priority=%d (%d), target is %d",
                     current_params.sched_priority, current_policy,
                     params.sched_priority);
    } else {
      inno_log_info("set thread priority to %d (%d)",
                    current_params.sched_priority, current_policy);
    }
  }
#endif  // __MINGW64__
  return;
}

#ifdef __MINGW64__
__inline static uint32_t __attribute__((__gnu_inline__,
                                        __always_inline__,
                                        __artificial__))
_mm_crc32_u8(uint32_t __C, uint8_t __V) {
  return __builtin_ia32_crc32qi(__C, __V);
}

__inline static uint32_t __attribute__((__gnu_inline__,
                                        __always_inline__,
                                        __artificial__))
_mm_crc32_u16(uint32_t __C, uint16_t __V) {
  return __builtin_ia32_crc32hi(__C, __V);
}

__inline static uint32_t __attribute__((__gnu_inline__,
                                        __always_inline__,
                                        __artificial__))
_mm_crc32_u32(uint32_t __C, uint32_t __V) {
  return __builtin_ia32_crc32si(__C, __V);
}
#endif

#ifdef _QNX_

/* from:
https://github.com/id-Software/DOOM-3-BFG/blob/master/neo/idlib/hashing/CRC32.cpp
Table of CRC-32's of all single-byte values */
static uint64_t crctable[256] = {
    0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
    0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
    0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
    0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
    0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
    0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
    0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
    0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
    0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
    0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
    0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
    0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
    0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
    0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
    0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
    0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
    0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
    0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
    0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
    0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
    0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
    0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
    0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
    0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
    0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
    0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
    0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
    0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
    0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
    0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
    0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
    0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
    0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
    0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
    0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
    0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
    0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
    0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
    0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
    0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
    0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
    0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
    0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
    0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
    0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
    0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
    0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
    0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
    0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
    0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
    0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
    0x2d02ef8dL};

uint32_t InnoUtils::crc32_do(uint32_t crc, const void *const buf,
                             const size_t buf_len) {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(buf);
  size_t bytes = buf_len;
  // Calculate the crc for the data
  while (bytes--) {
    crc = crctable[(crc ^ (*data++)) & 0xff] ^ (crc >> 8);
  }

  return crc;
}
#else
uint32_t InnoUtils::crc32_do(uint32_t crc, const void *const buf,
                             const size_t buf_len) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(buf);
  register uint32_t l = crc;

#if (defined(__i386__) || defined(__x86_64__)) && (!defined(__MINGW64__dd))
  for (size_t i = 0; i < (buf_len / sizeof(uint32_t)); ++i) {
    l = _mm_crc32_u32(l, *(const uint32_t *)p);
    p += sizeof(uint32_t);
  }
  if (buf_len & sizeof(uint16_t)) {
    l = _mm_crc32_u16(l, *(const uint16_t *)p);
    p += sizeof(uint16_t);
  }
  if (buf_len & sizeof(uint8_t)) {
    l = _mm_crc32_u8(l, *(const uint8_t *)p);
  }
#else

#if defined(__aarch64__)
  // from https://www.programmersought.com/article/13506713080/
#define CRC32X(crc, value) __asm__("crc32x %w[c], %w[c], %x[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32W(crc, value) __asm__("crc32w %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32H(crc, value) __asm__("crc32h %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32B(crc, value) __asm__("crc32b %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CX(crc, value) __asm__("crc32cx %w[c], %w[c], %x[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CW(crc, value) __asm__("crc32cw %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CH(crc, value) __asm__("crc32ch %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CB(crc, value) __asm__("crc32cb %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
  // Use local variables, use register optimization
  register size_t len = buf_len;

#define STEP1 do {                                              \
    CRC32CB(l, *p++);                                           \
    len--;                                                      \
} while (0)

#define STEP2 do {                                              \
    CRC32CH(l, *(uint16_t *)p);                                 \
    p += 2;                                                     \
    len -= 2;                                                   \
} while (0)

#define STEP4 do {                                              \
    CRC32CW(l, *(uint32_t *)p);                                 \
    p += 4;                                                     \
    len -= 4;                                                   \
} while (0)

#define STEP8 do {                                              \
    CRC32CX(l, *(uint64_t *)p);                                 \
    p += 8;                                                     \
    len -= 8;                                                   \
} while (0)

  // 512 way loop inline expansion
  while (len >= 512) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }
  // Use if to judge directly, the effect will be higher
  if (len >= 256) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 128) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 64) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 32) {
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 16) {
    STEP8; STEP8;
  }

  if (len >= 8) {
    STEP8;
  }

  if (len >= 4) {
    STEP4;
  }

  if (len >= 2) {
    STEP2;
  }

  if (len >= 1) {
    STEP1;
  }
#undef STEP8
#undef STEP4
#undef STEP2
#undef STEP1

#else
//  for (size_t i = 0; i < buf_len; ++i) {
//    l = (l << 5) + l + p[i];
//  }
#error "unsupported ARCH"

#endif  // ARM
#endif
  return l;
}
#endif

uint32_t InnoUtils::calculate_http_crc32(const char* buffer,
                                         uint32_t length,
                                         bool append) {
  uint32_t crc = InnoUtils::crc32_start();
  crc = InnoUtils::crc32_do(crc, buffer, length);
  if (append) {
    // add calc BEL 0x07 for separator
    crc = crc32_do(crc, kInnoSeparator, 2);
  }
  return crc32_end(crc);
}

int InnoUtils::verify_http_crc32(const char* buffer,
                                 const char* url) {
  std::string recv_buf(buffer);
  std::string recv_crc32;
  std::string context;
  uint32_t calc_crc32 = crc32_start();
  char ch_crc32[20] = {0};

  uint32_t buf_len = strlen(buffer);
  size_t pos = recv_buf.find("X-INNO-CRC32");
  if (pos != recv_buf.npos) {
    recv_crc32 = recv_buf.substr(pos + kInnoStrCRC32len,
                                 kInnoCRC32ValueLen);
    pos = recv_buf.find("\r\n\r\n");
    if (pos != recv_buf.npos) {
      context = recv_buf.substr(pos + 4, buf_len - pos - 4);
      if (url != NULL) {
        calc_crc32 = crc32_do(calc_crc32, url, strlen(url));
        // add calc BEL 0x07 for separator
        calc_crc32 = crc32_do(calc_crc32, kInnoSeparator, 2);
      }
      if (context.size() > 0) {
        calc_crc32 = crc32_do(calc_crc32, context.c_str(),
                              context.size());
      }
      calc_crc32 = crc32_end(calc_crc32);
      snprintf(ch_crc32, sizeof(ch_crc32), "%8x", calc_crc32);
      if (strcmp(ch_crc32, recv_crc32.c_str()) != 0) {
        inno_log_warning("CRC check failed. "
                         "calc_crc32 %s != recv_crc32 %s",
                         ch_crc32, recv_crc32.c_str());
        return -1;
      } else {
        return 0;
      }
    } else {
      inno_log_warning("Can't find \r\n\r\n");
      return -1;
    }
  } else {
    return 1;   // not check crc32 for Web
  }
}


/**
 * @Brief : check if the ip is valid
 * @param  ip
 * @return true  ip valid
 * @return false ip invalid
 */
bool InnoUtils::check_ip_valid(const char *ip) {
  std::string ip_address(ip);
  if (ip_address.empty() || ip_address == kInvalidIpAddress) {
    return false;
  }

  if (ip_address == kDefaultInterface)
    return true;

  struct in_addr new_ip;
  if (NetManager::inno_inet_pton(ip_address.c_str(), &new_ip) <= 0) {
    return false;
  }

  return true;
}

int InnoUtils::open_file(const char *filename, int flag_in, int mode) {
  int flag = flag_in;
#ifdef  __MINGW64__
  flag |= O_BINARY;
#endif
  int file_fd = open(filename, flag, mode);
  if (file_fd < 0) {
    inno_log_info("cannot open %s", filename);
    const char *ppwd = getenv("PWD");
    if (ppwd != NULL) {
      std::string pwd = ppwd;
      std::string file_fullpath = pwd + "/" + filename;
      file_fd = open(file_fullpath.c_str(), mode);
      if (file_fd < 0) {
        inno_log_error_errno("cannot open %s",
                             file_fullpath.c_str());
        return -1;
      } else {
        // pwd+file opened
        inno_log_info("open %s", file_fullpath.c_str());
        return file_fd;
      }
    } else {
      inno_log_error("cannot get pwd, cannot open file");
      return -3;
    }
  } else {
    inno_log_info("open %s", filename);
  }
  return file_fd;
}

#ifdef __MINGW64__
bool InnoUtils::is_socket_fd(int fd) {
  struct sockaddr_in addr;
  int ilen = sizeof(addr);
  int status = getsockname(fd, (struct sockaddr*)&addr, &ilen);
  if (status == 0) {
    return true;
  } else {
    return false;
  }
}
#endif

int InnoUtils::close_fd(int fd) {
#ifndef __MINGW64__
  return close(fd);
#else
  if (is_socket_fd(fd)) {
    inno_log_info("close socket, fd = %d", fd);
    return closesocket(fd);
  } else {
    inno_log_info("close file, fd = %d", fd);
    return close(fd);
  }
#endif
}

int InnoUtils::list_file(const std::string &path,
                         const std::string &pattern,
                         std::vector<std::string> *ret) {
  DIR *dp = ::opendir(path.c_str());
  if (dp == nullptr) {
    inno_log_error("open dir %s error", path.c_str());
    return -1;
  }
  struct dirent *entry;
  while ((entry = ::readdir(dp)) != nullptr) {
    if (strstr(entry->d_name, pattern.c_str())) {
      std::string entry_full_name = path +
        (InnoUtils::ends_with(path.c_str(), "/") ? "" : "/") + entry->d_name;
      ret->push_back(entry_full_name);
    }
  }
  ::closedir(dp);
  return 0;
}

std::string InnoUtils::get_current_time_str(const std::string& format) {
  time_t rawtime;
  struct tm *info;
  char temp[80];
  struct tm result_time;

  time(&rawtime);
#ifndef __MINGW64__
  info = localtime_r(&rawtime, &result_time);
#else
  info = localtime_s(&result_time, &rawtime) == 0 ?
         &result_time : NULL;
#endif
  strftime(temp, sizeof(temp), format.c_str(), info);
  return std::string(temp);
}

/**
 * 1. Create a SOCK_DGRAM socket(AF_INET, SOCK_DGRAM, 0)
 * 2. set reuse addr
 * 3. bind to input port
 * 4. set input opts
 * @param port
 * @param opts
 * @return -1 if failed. socket_fd if success.
 */
int InnoUdpHelper::bind(uint16_t port,
                        const std::vector<InnoUdpOpt> &opts) {
  int socket_fd = -1;
  struct sockaddr_in udp_listener_addr;
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    inno_log_error_errno("udp listener socket creation error %d", socket_fd);
    return -1;
  }

#ifndef  __MINGW64__
  int reuse = 1;
#else
  char reuse = 1;
#endif
  if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR,
                 &reuse, sizeof(reuse)) < 0) {
    inno_log_error("udp listener set reuse address error");
    close(socket_fd);
    return -1;
  }

  memset(&udp_listener_addr, 0, sizeof(udp_listener_addr));
  udp_listener_addr.sin_family = AF_INET;
  udp_listener_addr.sin_addr.s_addr = INADDR_ANY;
  udp_listener_addr.sin_port = htons(port);
  if (::bind(socket_fd, (const sockaddr*)&udp_listener_addr,
           sizeof(udp_listener_addr)) < 0) {
    inno_log_error_errno("failed to bind to port:%d", port);
    close(socket_fd);
    return -1;
  }

  for (auto & opt : opts) {
    int rs = setsockopt(socket_fd, opt.level, opt.optname,
                        reinterpret_cast<const char *>(opt.optval),
                        opt.optlen);
    if (rs < 0) {
      inno_log_info("opt: %d, %d, %p, %u, %s",
                    opt.level, opt.optname,
                    opt.optval, opt.optlen, opt.optname_str);
      inno_log_error_errno("setsockopt %s error %hu", opt.optname_str, port);
      close(socket_fd);
      return -1;
    }
  }

  return socket_fd;
}
}  // namespace innovusion
