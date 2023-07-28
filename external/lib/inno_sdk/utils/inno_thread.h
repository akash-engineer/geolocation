/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_INNO_THREAD_H_
#define UTILS_INNO_THREAD_H_

#include <pthread.h>
#include <stdint.h>
#include <time.h>

#if defined(_QNX_) || defined (__MINGW64__)
struct cpu_set_t;
#endif

namespace innovusion {
class InnoThread {
 public:
  InnoThread(const char *name, int priority,
             int worker_num,
             void *(*func_main)(void *),
             void *func_context,
             size_t cpusetsize,
             const cpu_set_t *cpuset);
  ~InnoThread();
  void start();
  void shutdown();
  bool has_shutdown() const {
    return shutdown_;
  }

 private:
  static void *inno_thread_func_(void *context);

 private:
  char *name_;
  int priority_;
  int worker_num_;
  pthread_t *threads_;
  void *(*func_main_)(void *);
  void *func_context_;
  size_t cpusetsize_;
  const cpu_set_t *cpuset_;
  pthread_mutex_t mutex_;
  pthread_cond_t cond_;
  bool shutdown_;
  size_t start_time_;
};
}  // namespace innovusion

#endif  // UTILS_INNO_THREAD_H_
