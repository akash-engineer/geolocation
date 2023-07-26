/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_LOG_H_
#define UTILS_LOG_H_

#if !(defined(_QNX_) || defined (__MINGW64__))
#include <execinfo.h>
#endif

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include "utils/inno_lidar_log.h"
#include "utils/consumer_producer.h"

namespace innovusion {
// define the format of log
// cross include and earlier declare
class AsyncLogManager;
typedef struct {
  enum InnoLogLevel level;
  char *head1_p;
  int   head1_len;
  char *head2_p;
  int   level_len;
  int   head2_len;
  char *body_p;
  int   body_len;
  char  payload[0];
} logContextInfo;

typedef int (*process_job)(logContextInfo *job, bool prefer);
/**
 * @class RotateLogFile :
 * When the log file is larger than the max length(rotate_file_size_limit),
 * a new file will be created to do this.  the name of log file will do the rotation.
 * this class only is used by the InnoLog.
 */
class RotateLogFile {
 public:
// constructor :  init properties
  RotateLogFile(const char *rotate_file_base_file,
             uint32_t rotate_file_number,
             uint64_t rotate_file_size_limit)
      : log_file_fd_(-1)
      , current_file_size_(0)
      , rotate_file_base_file_(rotate_file_base_file)
      , rotate_file_number_(rotate_file_number)
      , rotate_file_size_limit_(rotate_file_size_limit) {
    pthread_mutex_init(&mutex_, NULL);
  }
// Destructor
  ~RotateLogFile() {
    if (log_file_fd_ >= 0) {
      close(log_file_fd_);
      log_file_fd_ = -1;
    }
    pthread_mutex_destroy(&mutex_);
  }

  void write(const logContextInfo &info);

 private:
  int log_rotate_();

 private:
// Do the protection for rotation action
  pthread_mutex_t mutex_;
// Current log ID.  >0 means valid  -1 : invalid
  int log_file_fd_;
// Current file's size
  size_t current_file_size_;
// base name of rotation file
  std::string rotate_file_base_file_;
// the max number of rotation files
  uint32_t rotate_file_number_;
// the max size of file
  size_t rotate_file_size_limit_;
};

/**
 * @class : InnoLog
 * the InnLog will be a single instance per process.
 * it will be as the uniform API and provide to other process
 * to call the log function.
 */
class InnoLog {
 private:
// single instance
  InnoLog();
  ~InnoLog();
  // start and shutdown the async thread
  void start_async_log_thread_(void);
  void shutdown_async_log_thread_(void);
// for the exception call back
#ifndef __MINGW64__
  static void sighandler_(int sig, siginfo_t *sinfo, void *cont);
  static void print_backtrace_(int sig, struct sigcontext *ctx);
#endif

 public:
  // disable the default assign
  InnoLog(InnoLog const&) = delete;
  void operator=(InnoLog const&) = delete;

  static void setup_sig_handler();

  static InnoLog &get_instance(void) {
    static InnoLog instance;
    return instance;
  }
  /**
   * @brief :
   * @param  level : log level
   * @param  file  : file name
   * @param  line  : code line
   * @param  fmt   : format
   * @param  valist: params
   * @return int : 0 : succeed -1: failed
   */
  int log_v(enum InnoLogLevel level,
            bool discardable,
            const char *file, int line,
            const char *fmt,
            va_list valist);

  void set_logs(int out_fd, int error_fd,
                const char *rotate_file_base_file,
                uint32_t rotate_file_number,
                uint64_t rotate_file_size_limit,
                const char *rotate_file_base_file_err,
                uint32_t rotate_file_number_err,
                uint64_t rotate_file_size_limit_err,
                InnoLogCallback log_callback,
                void *ctx,
                bool do_async);

  void set_logs_callback(InnoLogCallback log_callback,
                void *ctx);

  static int log_printv(int fd, const logContextInfo &info);

  // add the static process function to the consumer of async log
  static int process(void *job, void *ctx, bool prefer);
  // print the status
  void asynclog_info();
  // add the for debug
  inline AsyncLogManager* get_asynclog_manager() {
    return asynclog_manager_p_;
  }
#ifdef ASYCLOG_UNITEST_ENABLE
  // add this for debug
  void set_process_hook_for_asynclog(process_job process_job_hook) {
    process_job_hook_ = process_job_hook;
  }
#endif

 public:
#ifndef __MINGW64__
  static struct sigaction olddisp_segv, olddisp_fpe, olddisp_bus, newdisp;
#endif

 private:
  // terminal log
  int log_out_fd_;
  int log_error_fd_;
  pthread_mutex_t log_out_mutex_;
  pthread_mutex_t log_err_mutex_;
  void log2terminal_(const logContextInfo &info);
  // file log
  RotateLogFile *rotate_out_;
  RotateLogFile *rotate_error_;
  void log2file_(const logContextInfo &info);
  // callback log
  InnoLogCallback log_callback_;
  void *log_callback_ctx_;
  pthread_rwlock_t log_callback_lock_;

  void log2Callback_(const logContextInfo &info);
  // AsyncLogManager entry
  AsyncLogManager  * asynclog_manager_p_;
  pthread_rwlock_t asynclog_lock_;
  bool asynclog_exist_ = false;
  int process_job_(logContextInfo *job, bool prefer);
#ifdef ASYCLOG_UNITEST_ENABLE
  // add this for debug
  process_job process_job_hook_ = NULL;
#endif
};

}  // namespace innovusion

#endif  // UTILS_LOG_H_
