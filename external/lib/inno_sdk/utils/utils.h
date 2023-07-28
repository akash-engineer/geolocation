/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_UTILS_H_
#define UTILS_UTILS_H_

#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <time.h>

#ifdef __MINGW64__
#include <ws2tcpip.h>
#endif

#ifdef _QNX_
#include <sys/socket.h>
#endif

#include <string>
#include <vector>
#include <queue>
#include <algorithm>

#include "./log.h"

namespace innovusion {
#if defined(_QNX_) || defined (__MINGW64__)
#define CLOCK_MONOTONIC_RAW (CLOCK_REALTIME)
#endif

#define SAFE_COPY_FIX_STRING_ARRAY(src, dest)                           \
  do {                                                                  \
    inno_log_verify(src && strlen(src) && strlen(src) < sizeof(dest),   \
                    "invalid copy");                                    \
    strncpy(dest, src, sizeof(dest));                                   \
    dest[sizeof(dest) - 1] = 0;                                         \
  } while (0)

const char* const kInvalidIpAddress = "0.0.0.0";
const char* const kDefaultInterface = "eth0";

class InnoUtils {
 private:
  static const char *white_space_;
  static const uint64_t kNsInSecond = 1000000000L;

 public:
  static const uint32_t  kInnoCRC32ValueLen = 8;
  static const uint32_t  kInnoStrCRC32len = 14;
  static const char* kInnoSeparator;

 public:
  static inline std::string *ltrim(std::string *str,
                                   const char *chars) {
    str->erase(0, str->find_first_not_of(chars));
    return str;
  }

  static inline std::string *rtrim(std::string *str,
                                   const char *chars) {
    str->erase(str->find_last_not_of(chars) + 1);
    return str;
  }

  static inline std::string *trim(std::string *str,
                                  const char *chars) {
    return ltrim(rtrim(str, chars), chars);
  }

  static inline std::string *trim_space(std::string *str) {
    return trim(str, white_space_);
  }

  static std::string *remove_all_space(std::string *str) {
    return remove_all_char(str, ' ');
  }

  static std::string *remove_all_char(std::string *str,
                                      const char c) {
    std::string::iterator end_pos = std::remove(str->begin(), str->end(), c);
    str->erase(end_pos, str->end());
    return str;
  }

  static std::string *remove_all_chars(std::string *str,
                                       const char *chars) {
    int i = 0;
    while (chars[i] != '\0') {
      remove_all_char(str, chars[i]);
      i++;
    }
    return str;
  }

  static std::vector<std::string> split(std::string raw_str,
                             const std::string &delimiter) {
    size_t pos = 0;
    std::string token;
    std::vector<std::string> splited_strings;
    while ((pos = raw_str.find(delimiter)) != std::string::npos) {
      token = raw_str.substr(0, pos);
      splited_strings.push_back(token);
      raw_str.erase(0, pos + delimiter.length());
    }
    splited_strings.push_back(raw_str);
    return splited_strings;
  }

  static inline bool is_unsinged_decimal_integer(const std::string &str) {
    return !str.empty() &&
           str.find_first_not_of("0123456789") == std::string::npos;
  }

  static void set_self_thread_priority(int priority);

  static inline uint64_t get_time_ns(clockid_t clk_id) {
    struct timespec spec;
    clock_gettime(clk_id, &spec);
    return spec.tv_sec * 1000000000L + spec.tv_nsec;
  }

  static inline uint64_t get_time_us(clockid_t clk_id) {
    return get_time_ns(clk_id) / 1000;
  }

  static inline uint64_t get_time_ms(clockid_t clk_id) {
    return get_time_ns(clk_id) / 1000000;
  }

  static void us_to_timespec(uint64_t us, struct timespec *spec) {
    spec->tv_sec = us / 1000000;
    spec->tv_nsec = (us % 1000000) * 1000;
  }

  static bool ends_with(const char *str, const char *suffix) {
    if (!str || !suffix) {
      return false;
    }
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix > lenstr) {
      return false;
    }
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
  }

  static bool start_with(const char *str, const char *prefix) {
    if (!str || !prefix) {
      return false;
    }
    size_t lenstr = strlen(str);
    size_t lenprefix = strlen(prefix);
    if (lenprefix > lenstr) {
      return false;
    }
    return strncmp(str, prefix, lenprefix) == 0;
  }

  static inline uint32_t crc32_start() {
    return 0xffffffffu;
  }

  static inline uint32_t crc32_end(uint32_t crc) {
    return crc ^ 0xffffffffu;
  }

  static uint32_t crc32_do(uint32_t crc, const void* buf,
                           const size_t len);

  static uint32_t calculate_http_crc32(const char* buffer,
                                       uint32_t length,
                                       bool append = false);

  static int verify_http_crc32(const char* buffer,
                               const char* url);

  static int open_file(const char *filename,
                       int flag_in, int mode);

  static bool check_ip_valid(const char *ip);

#ifdef __MINGW64__
  static bool is_socket_fd(int fd);
#endif
  static int close_fd(int fd);

  static inline uint64_t get_timestamp_ns(timespec spec) {
    return spec.tv_sec * kNsInSecond + spec.tv_nsec;
  }

  static int list_file(const std::string &path,
                       const std::string &pattern,
                       std::vector<std::string> *ret);

  static std::string get_current_time_str(const std::string& format
                                            = "%m_%d_%Y_%H_%M_%S");

  static void run_command(const char *cmd, char *buffer, const int buf_size) {
    inno_log_verify(cmd && buffer && buf_size > 0,
                          "Run command conditions error");
    buffer[0] = '\0';
    FILE *fp = popen(cmd, "r");
    if (fp == NULL) {
      inno_log_error("fail to exec popen(%s)", cmd);
      return;
    }
    usleep(10);
    size_t numread = fread(buffer, 1, buf_size, fp);
    pclose(fp);
    if (numread > 0) {
      buffer[buf_size - 1] = '\0';
    }
  }
};

class InnoMean {
 public:
  InnoMean() {
    reset();
  }

  ~InnoMean() {
  }

 public:
  void reset() {
    sum_ = 0;
    sum2_ = 0;
    count_ = 0;
    delta_max_ = 0;
    max_ = 0;
    min_ = 0;
  }

  double mean() const {
    uint64_t c = count_;
    double s = sum_;
    if (c > 0) {
      return s / c;
    } else {
      return 0;
    }
  }

  uint64_t count() const {
    return count_;
  }

  double std_dev() const {
    uint64_t c = count_;
    double s = sum_;
    double s2 = sum2_;
    if (c > 0) {
      double a = s / c;
      double d = s2 / c - a * a;
      if (d > 0) {
        return sqrt(d);
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  }

  double max_delta() const {
    return delta_max_;
  }

  double max() const {
    return max_;
  }

  double min() const {
    return min_;
  }

  inline void add(double value) {
    if (count_ > 0) {
      double delta = fabs(value - pre_);
      if (delta_max_ < delta) {
        delta_max_ = delta;
      }
      if (max_ < value) {
        max_ = value;
      }
      if (min_ > value) {
        min_ = value;
      }
    } else {
      max_ = value;
      min_ = value;
    }
    count_++;
    sum_ += value;
    sum2_ += value * value;
    pre_ = value;
  }

 private:
  double delta_max_;
  double max_;
  double min_;
  double pre_;
  double sum_;
  double sum2_;
  uint64_t count_;
};

class InnoMeanLite {
 public:
  InnoMeanLite() {
    reset();
  }

  ~InnoMeanLite() {
  }

 public:
  void reset() {
    sum_ = 0;
    sum2_ = 0;
    count_ = 0;
  }

  double mean() const {
    int64_t c = count_;
    double s = sum_;
    if (c > 0) {
      return s / c;
    } else {
      return 0;
    }
  }

  int64_t count() const {
    return count_;
  }

  double std_dev() const {
    int64_t c = count_;
    double s = sum_;
    double s2 = sum2_;
    if (c > 0) {
      double a = s / c;
      double d = s2 / c - a * a;
      if (d > 0) {
        return sqrt(d);
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  }

  inline void add(int64_t value) {
    count_++;
    sum_ += value;
    sum2_ += value * value;
  }

  inline void subtract(int64_t value) {
    count_--;
    sum_ -= value;
    sum2_ -= value * value;
  }

 private:
  int64_t sum_;
  int64_t sum2_;
  int64_t count_;
};

class InnoSlidingMean {
 public:
  explicit InnoSlidingMean(int max_q_len = 15) {
    max_q_len_ = max_q_len;
  }

  ~InnoSlidingMean() {
  }

  inline void add(int64_t value) {
    mean_.add(value);
    q_.push(value);
    while (q_.size() > max_q_len_) {
      int64_t front = q_.front();
      q_.pop();
      mean_.subtract(front);
    }
  }

  inline void pop() {
    if (q_.empty()) {
      return;
    }
    int front = q_.front();
    q_.pop();
    mean_.subtract(front);
  }

  inline double mean() {
    return mean_.mean();
  }

  inline double std_dev() {
    return mean_.std_dev();
  }

  inline void set_max_q_len(int len) {
    max_q_len_ = len;
  }

  inline int size() {
    return q_.size();
  }

  inline bool is_full() {
    return q_.size() >= max_q_len_;
  }

  inline bool is_empty() {
    return q_.empty();
  }

  inline void clear() {
    while (!q_.empty()) {
      q_.pop();
    }
    mean_.reset();
  }

 private:
  std::queue<int> q_;
  size_t max_q_len_;
  InnoMeanLite mean_;
};

class InnoUdpOpt {
 public:
  InnoUdpOpt(int level, int optname, const void *optval,
             socklen_t optlen, const char *optname_str) {
    this->level = level;
    this->optname = optname;
    this->optval = optval;
    this->optlen = optlen;
    this->optname_str = optname_str;
  }
 public:
  int level;
  int optname;
  const void *optval;
  socklen_t optlen;
  const char *optname_str;
};

class InnoUdpHelper {
 public:
  static int bind(uint16_t port, const std::vector<InnoUdpOpt> &opts);
  static int bind(const char *ip, uint16_t port,
                  const std::vector<InnoUdpOpt> &opts);
  static bool is_multicast_ip_addr(std::string ip) {
    std::vector<std::string> ip_segs = InnoUtils::split(ip, ".");
    if (ip_segs.size() != 4) {
      return false;
    }
    int i = atoi(InnoUtils::split(ip, ".")[0].c_str());
    return i >= 224 && i <= 239;
  }
};
}  // namespace innovusion

#endif  // UTILS_UTILS_H_
