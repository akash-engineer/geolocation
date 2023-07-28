/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef UTILS_INNO_LIDAR_LOG_H_
#define UTILS_INNO_LIDAR_LOG_H_

#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#ifdef __MINGW64__
// #  define PRI_SIZEU "I64u"
// #  define PRI_SIZED "I64d"
#  define PRI_SIZEU "I64u"
#  define PRI_SIZED "I64d"
#  define PRI_SIZEX "I64x"
#  define PRI_SIZELU "llu"
#else
#  define PRI_SIZEU "zu"
#  define PRI_SIZED "zd"
#  define PRI_SIZEX "zx"
#  define PRI_SIZELU "lu"
#endif

extern "C" {
  enum InnoLogLevel {
    INNO_LOG_LEVEL_FATAL = 0,
    INNO_LOG_LEVEL_CRITICAL = 1,
    INNO_LOG_LEVEL_ERROR = 2,
    INNO_LOG_LEVEL_TEMP = 3,
    INNO_LOG_LEVEL_WARNING = 4,
    INNO_LOG_LEVEL_DEBUG = 5,
    INNO_LOG_LEVEL_INFO = 6,
    INNO_LOG_LEVEL_TRACE = 7,
    INNO_LOG_LEVEL_DETAIL = 8,
    INNO_LOG_LEVEL_MAX = 9,
  };

  /*
   *  @param callback context
   *  @param level Log level.
   *  @param header1 First log message header, e.g.
   *         '2018-12-06 23:54:09'
   *  @param header2 Second log message header, e.g.
   *       '[ERROR] 7252 rawdata.cpp:3259'
   *  @param msg Log message body.
   *  @return Void.
   */
  typedef void (*InnoLogCallback)(void *ctx,
                                  enum InnoLogLevel level,
                                  const char *header1,
                                  const char *header2,
                                  const char *msg);

  extern enum InnoLogLevel inno_log_level_g;
  extern const char *inno_log_header_g[];

  void inno_log_print(enum InnoLogLevel,
                      bool discardable,
                      const char *file, int line,
                      const char *fmt, ...)
      __attribute__((format(printf, 5, 6)));

  void inno_fprintf(int fd,
                    const char *fmt, ...)
      __attribute__((format(printf, 2, 3)));
};

#define ABORT() abort()

#if defined(_QNX_) || defined (__MINGW64__)
struct cpu_set_t;
#endif

#define inno_ignore_variable(a) do {             \
    a = a;                                       \
  } while (0)

#ifdef NDEBUG
#define inno_log_assert(_condition, ...) do {} while (0)
#else
#define inno_log_assert(_condition, ...)                                \
  do {                                                                  \
    if (!(_condition)) {                                                \
      inno_log_with_level(INNO_LOG_LEVEL_FATAL,                         \
                          "@@@@@@@!!!!@@@@@@ condition check failed: (" \
                          #_condition ")");                             \
      inno_log_with_level(INNO_LOG_LEVEL_CRITICAL,                      \
                          __VA_ARGS__);                                 \
      ABORT();                                                          \
    }                                                                   \
  } while (0)
#endif

#define inno_log_verify_no_print(_condition, ...)                       \
  do {                                                                  \
    if (!(_condition)) {                                                \
      ABORT();                                                          \
    }                                                                   \
  } while (0)

#define inno_log_verify inno_log_panic_if_not
#define inno_log_panic_if_not(_condition, ...)                           \
  do {                                                                   \
    if (!(_condition)) {                                                 \
      inno_log_with_level(INNO_LOG_LEVEL_FATAL,                          \
                          "@@@@@@@!!!!@@@@@@@ condition check failed: (" \
                          #_condition ")");                              \
      inno_log_with_level(INNO_LOG_LEVEL_CRITICAL,                       \
                          __VA_ARGS__);                                  \
      ABORT();                                                           \
    }                                                                    \
  } while (0)

#define inno_log_verify_simple(_condition, ...)                          \
  do {                                                                   \
    if (!(_condition)) {                                                 \
      inno_log_with_level(INNO_LOG_LEVEL_CRITICAL,                       \
                          "@@@@@@@!!!!@@@@@@@ condition check failed: (" \
                          #_condition ")");                              \
      inno_log_with_level(INNO_LOG_LEVEL_CRITICAL,                       \
                          __VA_ARGS__);                                  \
      ABORT();                                                           \
    }                                                                    \
  } while (0)

#define inno_log_panic(...)                     \
  do {                                          \
    inno_log_with_level(INNO_LOG_LEVEL_FATAL,   \
                        __VA_ARGS__);           \
    ABORT();                                    \
  } while (0)

#define inno_log_NOT_IMPLEMENTED()              \
  do {                                          \
    inno_log_panic("NOT IMPLEMENTED%s", "");    \
  } while (0)

#define inno_log_fatal(...)                     \
  do {                                          \
    inno_log_with_level(INNO_LOG_LEVEL_FATAL,   \
                        __VA_ARGS__);           \
  } while (0)

#define inno_log_error(...)                                \
  do {                                                     \
    if (inno_log_level_g >= INNO_LOG_LEVEL_ERROR) {        \
      inno_log_with_level(INNO_LOG_LEVEL_ERROR,            \
                          __VA_ARGS__);                    \
    }                                                      \
  } while (0)

#define inno_log_error_errno(_fmt, ...)                         \
  do {                                                          \
    if (inno_log_level_g >= INNO_LOG_LEVEL_ERROR) {             \
      inno_log_with_level(INNO_LOG_LEVEL_ERROR,                 \
                          "strerror: '%s' " _fmt,               \
                          strerror(errno), __VA_ARGS__);        \
    }                                                           \
  } while (0)

#define inno_log_temp(...)                              \
  do {                                                  \
    if (inno_log_level_g > INNO_LOG_LEVEL_ERROR) {      \
      inno_log_with_level(INNO_LOG_LEVEL_TEMP,          \
                          __VA_ARGS__);                 \
    }                                                   \
  } while (0)

#define inno_log_warning(...)                           \
  do {                                                  \
    if (inno_log_level_g >= INNO_LOG_LEVEL_WARNING) {   \
      inno_log_with_level(INNO_LOG_LEVEL_WARNING,       \
                          __VA_ARGS__);                 \
    }                                                   \
  } while (0)

#define inno_log_warning_errno(_fmt, ...)                       \
  do {                                                          \
    if (inno_log_level_g >= INNO_LOG_LEVEL_WARNING) {           \
      inno_log_with_level(INNO_LOG_LEVEL_WARNING,               \
                          "strerror: '%s' " _fmt,               \
                          strerror(errno), __VA_ARGS__);        \
    }                                                           \
  } while (0)

#define inno_log_info(...)                                    \
  do {                                                        \
    if (inno_log_level_g >= INNO_LOG_LEVEL_INFO) {            \
      inno_log_with_level(INNO_LOG_LEVEL_INFO, __VA_ARGS__);  \
    }                                                         \
  } while (0)

#define inno_log_debug(...)                             \
  do {                                                  \
    if (inno_log_level_g >= INNO_LOG_LEVEL_DEBUG) {     \
      inno_log_with_level(INNO_LOG_LEVEL_DEBUG,         \
                          __VA_ARGS__);                 \
    }                                                   \
  } while (0)

#ifdef NDEBUG
#define inno_log_trace(...) do {} while (0)
#else
#define inno_log_trace(...)                             \
  do {                                                  \
    if (inno_log_level_g >= INNO_LOG_LEVEL_TRACE) {     \
      inno_log_with_level(INNO_LOG_LEVEL_TRACE,         \
                          __VA_ARGS__);                 \
    }                                                   \
  } while (0)
#endif

#ifdef NDEBUG
#define inno_log_detail(...) do {} while (0)
#else
#define inno_log_detail(...)                             \
  do {                                                   \
    if (inno_log_level_g >= INNO_LOG_LEVEL_DETAIL) {     \
      inno_log_with_level(INNO_LOG_LEVEL_DETAIL,         \
                          __VA_ARGS__);                  \
    }                                                    \
  } while (0)
#endif

#define inno_log_with_level(_level, ...)                 \
  do {                                                   \
    inno_log_print(_level, true, __FILE__,               \
                   __LINE__, __VA_ARGS__);               \
  } while (0)

#define inno_log_with_level_no_discard(_level, ...)      \
  do {                                                   \
    inno_log_print(_level, false, __FILE__,              \
                   __LINE__, __VA_ARGS__);               \
  } while (0)

#define inno_log_3th_program(_level, from, ...)            \
  do {                                                     \
    inno_log_print(_level, true, from, 7999, __VA_ARGS__); \
  } while (0)

#endif  // UTILS_INNO_LIDAR_LOG_H_
