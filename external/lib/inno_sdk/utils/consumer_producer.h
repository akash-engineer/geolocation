/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_CONSUMER_PRODUCER_H_
#define UTILS_CONSUMER_PRODUCER_H_

#include <pthread.h>
#include <stdint.h>
#include <time.h>

#include "./inno_thread.h"

namespace innovusion {
typedef int (*ConsumeFunc)(void *job, void *context, bool prefer);

class ConsumerProducer {
 private:
  enum Priority {
    PRIORITY_LOW = 0,
    PRIORITY_HIGH = 1,
    PRIORITY_MAX = 2,
  };

 private:
  class Job {
   public:
    Job()
        : job(NULL)
        , timestamp(0)
        , job_id(0)
        , not_discardable(false) {
    }
    Job(void *j, size_t t, uint64_t i, bool d)
        : job(j)
        , timestamp(t)
        , job_id(i)
        , not_discardable(d) {
    }

    void *job;
    size_t timestamp;
    uint64_t job_id;
    bool not_discardable;
  };

  class CpQueue {
   public:
    CpQueue(const char *name, int queue_size, bool allow_log);
    ~CpQueue();
    bool is_full() const {
      return full_;
    }
    bool is_empty() const {
      return empty_;
    }
    int queue_length() const {
      return count_;
    }
    void add(const Job &job);
    int pop(Job *job, bool peek_only = false);
    inline int peek(Job *job) {
      return pop(job, true);
    }

   private:
    inline bool do_log_() const {
      return allow_log_;
    }

    char *name_;
    int head_, tail_;
    bool full_, empty_;
    int queue_size_, count_;
    int64_t done_;
    Job *jobs_;
    bool allow_log_;
  };

 public:
  static void *consumer_thread_func_(void *context);

 public:
  ConsumerProducer(const char *name, int priority,
                   int worker_num, ConsumeFunc consume_func,
                   void *consume_context,
                   int queue_size, int prefer_queue_size,
                   int hi_cp_queue_size,
                   size_t cpusetsize,
                   const cpu_set_t *cpuset,
                   bool allow_log_ = true);
  ~ConsumerProducer();
  void start();
  inline int add_job(void *in, bool high_priority = false,
                     bool not_discardable = false) {
    return add_job_do_(in, high_priority, NULL, not_discardable);
  }
  int add_job_wait_done(void *in, bool high_priority = false);
  void shutdown();
  void flush_and_pause();
  void resume();
  inline int queue_length() const {
    return queue_.queue_length();
  }
  inline int max_queue_length() const {
    return max_queue_len_;
  }
  inline pthread_t *get_threads() {
    return threads_;
  }
  inline size_t dropped_job_count() const {
    size_t r = 0;
    for (int i = 0; i < PRIORITY_MAX; i++) {
      r += dropped_job_[i];
    }
    return r;
  }
  inline size_t blocked_job_count() const {
    size_t r = 0;
    for (int i = 0; i < PRIORITY_MAX; i++) {
      r += blocked_job_[i];
    }
    return r;
  }
  void print_stats(void);
  void get_stats_string(char *buf, size_t buf_size);

 private:
  uint64_t assign_job_id_(int idx) {
    return ++job_id_[idx];
  }
  void update_done_job_id_(int idx, uint64_t id) {
    finished_job_id_[idx] = id;
  }
  int add_job_do_(void *in, bool high_priority,
                  uint64_t *job_id_out, bool not_discardable);
  int get_job_(Job *job,
               enum Priority *priorty);
  inline bool do_log_() const {
    return allow_log_;
  }
  uint64_t added_job_total_();
  uint64_t finished_job_total_();
  uint64_t dropped_job_total_();

 private:
  char *name_;
  int priority_;
  CpQueue queue_;
  bool blocking_;
  bool paused_;
  int prefer_queue_size_;
  CpQueue hi_queue_;
  int worker_num_;
  pthread_t *threads_;
  ConsumeFunc consume_func_;
  void *consume_context_;
  pthread_mutex_t mutex_;
  pthread_cond_t not_full_, not_empty_, job_done_cond_;
  int started_;
  bool shutdown_;
  size_t cpusetsize_;
  const cpu_set_t *cpuset_;
  bool allow_log_;

  size_t start_time_;
  size_t total_wait_time_[PRIORITY_MAX];
  size_t total_process_time_[PRIORITY_MAX];
  size_t total_drop_time_[PRIORITY_MAX];
  size_t added_job_[PRIORITY_MAX];
  size_t finished_job_[PRIORITY_MAX];
  size_t blocked_job_[PRIORITY_MAX];
  size_t dropped_job_[PRIORITY_MAX];
  uint64_t job_id_[PRIORITY_MAX];
  uint64_t finished_job_id_[PRIORITY_MAX];
  int32_t max_queue_len_;
  size_t last_active_time_;
  size_t last_elapse_time_;
  int pid_{0};
};
}  // namespace innovusion

#endif  // UTILS_CONSUMER_PRODUCER_H_
