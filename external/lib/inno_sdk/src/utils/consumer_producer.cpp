/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "utils/consumer_producer.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#if !(defined(_QNX_) || defined (__MINGW64__))
#include <sys/syscall.h>
#endif
#include <sys/types.h>

#include "utils/log.h"
#include "utils/utils.h"

namespace innovusion {
ConsumerProducer::CpQueue::CpQueue(const char *name,
                                   int queue_size,
                                   bool allow_log) {
  name_ = strdup(name);
  done_ = 0;
  empty_ = true;
  full_ = false;
  count_ = 0;
  head_ = 0;
  tail_ = 0;
  queue_size_ = queue_size;
  jobs_ = new Job[queue_size];
  allow_log_ = allow_log;
  if (do_log_()) {
    inno_log_verify(name_, "name %s", name);
    inno_log_verify(jobs_, "%s jobs", name_);
  }
}

ConsumerProducer::CpQueue::~CpQueue() {
  if (do_log_()) {
    inno_log_verify(count_ == 0, "%s count = %d", name_, count_);
  }
  delete [] jobs_;
}

void ConsumerProducer::CpQueue::add(const Job &job) {
  jobs_[tail_] = job;

  tail_++;
  if (tail_ == queue_size_) {
    tail_ = 0;
  }
  if (tail_ == head_) {
    full_ = true;
  }
  empty_ = false;
  count_++;
  if (do_log_()) {
    inno_log_verify(count_ <= queue_size_,
                    "%s count = %d queue_size_ = %d",
                    name_, count_, queue_size_);
  }

  return;
}

int ConsumerProducer::CpQueue::pop(Job *job,
                                   bool peek_only) {
  int ret  = count_;
  if (empty_) {
    if (do_log_()) {
      inno_log_verify(count_ == 0,
                      "%s count = %d",
                      name_, count_);
    }
    return 0;
  } else {
    if (do_log_()) {
      inno_log_verify(count_ > 0,
                      "%s count = %d",
                      name_, count_);
    }
  }
  *job = jobs_[head_];
  if (peek_only) {
    return ret;
  }

  head_++;
  if (head_ == queue_size_) {
    head_ = 0;
  }
  count_--;
  if (head_ == tail_) {
    empty_ = true;
    if (do_log_()) {
      inno_log_verify(count_ == 0,
                      "%s count = %d",
                      name_, count_);
    }
  }
  full_ = false;

  return ret;
}

ConsumerProducer::ConsumerProducer(const char *name,
                                   int priority,
                                   int worker_num, ConsumeFunc consume_func,
                                   void *consume_context,
                                   int queue_size, int prefer_queue_size,
                                   int hi_cp_queue_size,
                                   size_t cpusetsize,
                                   const cpu_set_t *cpuset,
                                   bool allow_log)
    : priority_(priority)
    , queue_(name, queue_size, allow_log)
    , prefer_queue_size_(prefer_queue_size)
    , hi_queue_(name, hi_cp_queue_size, allow_log)
    , worker_num_(worker_num)
    , consume_func_(consume_func)
    , consume_context_(consume_context)
    , started_(0)
    , shutdown_(true)
    , cpusetsize_(cpusetsize)
    , cpuset_(cpuset)
    , allow_log_(allow_log) {
  name_ = strdup(name);
  if (do_log_()) {
    inno_log_verify(name_, "%s name", name);
  }
  pthread_mutex_init(&mutex_, NULL);
  pthread_cond_init(&not_full_, NULL);
  pthread_cond_init(&not_empty_, NULL);
  pthread_cond_init(&job_done_cond_, NULL);
  if (prefer_queue_size_ <= 0) {
    prefer_queue_size_ = queue_size;
    blocking_ = true;
  } else {
    blocking_ = false;
  }
  paused_ = false;
  threads_ = NULL;
  for (int i = 0; i < PRIORITY_MAX; i++) {
    total_wait_time_[i] = 0;
    total_process_time_[i] = 0;
    total_drop_time_[i] = 0;
    added_job_[i] = 0;
    finished_job_[i] = 0;
    blocked_job_[i] = 0;
    dropped_job_[i] = 0;
    job_id_[i] = 0;
    finished_job_id_[i] = 0;
  }
  max_queue_len_ = 0;
  start_time_ = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
  last_active_time_ = 0;
  last_elapse_time_ = 0;
}

ConsumerProducer::~ConsumerProducer() {
  if (do_log_()) {
    inno_log_verify(shutdown_ && !paused_,
                    "%s shutdown=%d paused=%d started=%d",
                    name_, shutdown_, paused_, started_);
  }
  pthread_mutex_destroy(&mutex_);
  pthread_cond_destroy(&not_full_);
  pthread_cond_destroy(&not_empty_);
  pthread_cond_destroy(&job_done_cond_);
  free(name_);
  name_ = NULL;
}

void *ConsumerProducer::consumer_thread_func_(void *context) {
  ConsumerProducer *cp = reinterpret_cast<ConsumerProducer *>(context);
#if !(defined(_QNX_) || defined (__MINGW64__))
  pid_t pid = 0;
  if (cp->cpuset_ && cp->cpusetsize_) {
    const pthread_t tid = pthread_self();
    pthread_setaffinity_np(tid, cp->cpusetsize_,
                           cp->cpuset_);
  }
  pid = syscall(SYS_gettid);
#else
  uint32_t pid = 0;
#endif
  inno_log_info("thread %s starts. pid=%d target_priority=%d",
                cp->name_, pid, cp->priority_);
  cp->pid_ = pid;
  InnoUtils::set_self_thread_priority(cp->priority_);

  while (1) {
    enum Priority priority;
    Job job;
    int r = cp->get_job_(&job, &priority);
    size_t timestamp_poped = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    bool is_prefered;
    int idx = 0;
    if (r == 0) {
      break;
    }
    if (priority == PRIORITY_LOW) {
      is_prefered = (r <= cp->prefer_queue_size_);
      idx = 0;
    } else {
      // high priority queue
      is_prefered = true;
      idx = 1;
    }
    cp->consume_func_(job.job, cp->consume_context_, is_prefered);

    size_t timestamp_done = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    pthread_mutex_lock(&cp->mutex_);
    cp->total_wait_time_[priority] += (timestamp_poped - job.timestamp);
    if (is_prefered) {
      cp->finished_job_[priority]++;
      cp->total_process_time_[priority] += (timestamp_done - timestamp_poped);
    } else {
      cp->dropped_job_[priority]++;
      cp->total_drop_time_[priority] += (timestamp_done - timestamp_poped);
    }
    cp->update_done_job_id_(idx, job.job_id);
    pthread_mutex_unlock(&cp->mutex_);
    pthread_cond_broadcast(&cp->job_done_cond_);
    /*
    inno_log_debug("%s %p out enq=%.4f deq=%.4f done=%.4f", cp->name_,
                   job.job,
                   job.timestamp/1000000000.0,
                   timestamp_poped/1000000000.0,
                   timestamp_done/1000000000.0);
    */
    // yield cpu that firmware can switch mode in time
    // "Use of sched_yield()
    //  with nondeterministic scheduling policies such as SCHED_OTHER is
    //  unspecified and very likely means your application design is
    //  broken."
    if (cp->priority_ > 0) {
      sched_yield();
    }
  }
  return NULL;
}

void ConsumerProducer::start() {
  pthread_mutex_lock(&mutex_);
  started_++;
  shutdown_ = false;
  pthread_mutex_unlock(&mutex_);
  // pthread_cond_broadcast(&not_empty_);
  // creates threads
  threads_ = reinterpret_cast<pthread_t *>\
             (malloc(worker_num_ *sizeof(pthread_t)));
  for (int i = 0; i < worker_num_; i++) {
    pthread_create(&threads_[i], NULL, consumer_thread_func_, this);
  }
}

void ConsumerProducer::shutdown() {
  print_stats();
  pthread_mutex_lock(&mutex_);
  if (do_log_()) {
    inno_log_verify(!shutdown_, "%s shutdown=%d started=%d",
                    name_, shutdown_, started_);
  }
  shutdown_ = true;
  pthread_mutex_unlock(&mutex_);
  pthread_cond_broadcast(&not_empty_);
  pthread_cond_broadcast(&not_full_);
  // join threads
  for (int i = 0; i < worker_num_; i++) {
    void *ret;
    pthread_join(threads_[i], &ret);
  }
  free(threads_);
  threads_ = NULL;
}

uint64_t ConsumerProducer::added_job_total_() {
  uint64_t ret = 0;
  for (int i = 0; i < PRIORITY_MAX; i++) {
    ret += added_job_[i];
  }
  return ret;
}

uint64_t ConsumerProducer::finished_job_total_() {
  uint64_t ret = 0;
  for (int i = 0; i < PRIORITY_MAX; i++) {
    ret += finished_job_[i];
  }
  return ret;
}

uint64_t ConsumerProducer::dropped_job_total_() {
  uint64_t ret = 0;
  for (int i = 0; i < PRIORITY_MAX; i++) {
    ret += dropped_job_[i];
  }
  return ret;
}

void ConsumerProducer::flush_and_pause() {
  pthread_mutex_lock(&mutex_);
  if (do_log_()) {
    inno_log_verify(!paused_, "paused_");
  }
  paused_ = true;
  while (added_job_total_() !=
         finished_job_total_() +
         dropped_job_total_()) {
    if (do_log_()) {
      inno_log_verify(added_job_total_() >
                      finished_job_total_() +
                      dropped_job_total_(),
                      "%" PRI_SIZEU " should be bigger %"
                      PRI_SIZEU " + %" PRI_SIZEU "",
                      added_job_total_(),
                      finished_job_total_(),
                      dropped_job_total_());
    }
    pthread_cond_wait(&job_done_cond_, &mutex_);
  }
  // return with mutex locked
  return;
}

void ConsumerProducer::resume() {
  if (do_log_()) {
    inno_log_verify(paused_, "paused_");
  }
  paused_ = false;
  pthread_mutex_unlock(&mutex_);
  // wake up producer
  pthread_cond_broadcast(&not_full_);
}

int ConsumerProducer::add_job_wait_done(void *in,
                                        bool high_priority) {
  uint64_t job_id;
  if (do_log_()) {
    inno_log_verify(worker_num_ == 1,
                    "add_job_until_done only support 1 worker");
  }
  int r = add_job_do_(in, high_priority, &job_id, true);
  pthread_mutex_lock(&mutex_);
  int idx = high_priority ? 1 : 0;
  while (job_id > finished_job_id_[idx]) {
    pthread_cond_wait(&job_done_cond_, &mutex_);
  }
  pthread_mutex_unlock(&mutex_);
  return r;
}
// 0: normal enqueue 1: block enqueue 2: give up enqueue
// 3: discard head and enqueue
int ConsumerProducer::add_job_do_(void *job, bool high_priority,
                                  uint64_t *job_id_out,
                                  bool not_discardable) {
  bool done = false;
  int ret = 0;
  CpQueue &queue = high_priority ? hi_queue_ : queue_;
  int idx;
  bool blocking;
  size_t timestamp_in = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
  uint64_t job_id = 0;
  /*
  if (do_log_()) {
    inno_log_debug("%s %p in enq=%.4f", name_,
      job, timestamp_in/1000000000.0);
  }
  */
  if (high_priority) {
    idx = 1;
    blocking = true;
  } else {
    idx = 0;
    blocking = blocking_ || not_discardable;
  }
  while (!done) {
    pthread_mutex_lock(&mutex_);
    if (paused_) {
      pthread_cond_wait(&not_full_, &mutex_);
      pthread_mutex_unlock(&mutex_);
      continue;
    }
    // cannot be shutdown while any producer is still alive
    if (do_log_()) {
      inno_log_verify(!shutdown_, "%s shutdown=%d started=%d",
                      name_, shutdown_, started_);
    }
    if (queue.is_full()) {
      if (blocking) {
        ret = 1;
        pthread_cond_wait(&not_full_, &mutex_);
        pthread_mutex_unlock(&mutex_);
        continue;
      } else {
        Job discard_job;
        int r = queue.peek(&discard_job);
        if (discard_job.not_discardable) {
          // cannot drop the one in the queue
          // have to discard itself
          added_job_[idx]++;
          dropped_job_[idx]++;
          pthread_mutex_unlock(&mutex_);
          consume_func_(job, consume_context_, false);
          // don't need to signal job_done_cond_
          // give up enqueue
          ret = 2;
          done = true;
        } else {
          r = queue.pop(&discard_job);
          dropped_job_[idx]++;
          if (do_log_()) {
            inno_log_verify(r,
                            "%s r = %d",
                            name_, r);
          }
          update_done_job_id_(idx, discard_job.job_id);
          pthread_mutex_unlock(&mutex_);
          consume_func_(discard_job.job, consume_context_, false);
          pthread_cond_broadcast(&job_done_cond_);
          // discard the head and enqueue
          ret = 3;
        }
      }
    } else {
      job_id = assign_job_id_(idx);
      Job j(job, timestamp_in, job_id, not_discardable);
      queue.add(j);
      int l = queue_.queue_length();
      if (max_queue_len_ < l) {
        max_queue_len_ = l;
      }
      added_job_[idx]++;
      pthread_mutex_unlock(&mutex_);
      pthread_cond_broadcast(&not_empty_);
      done = true;
    }
  }
  // only update the block case
  if (ret == 1) {
    blocked_job_[idx] += ret;
  }

  if (job_id_out) {
    *job_id_out = job_id;
  }
  return ret;
}

int ConsumerProducer::get_job_(Job *job,
                               enum Priority *priority) {
  for (;;) {
    pthread_mutex_lock(&mutex_);
    if (queue_.is_empty() && hi_queue_.is_empty() && !shutdown_) {
      pthread_cond_wait(&not_empty_, &mutex_);
    } else {
    }
    int ret = 0;
    ret = hi_queue_.pop(job);
    if (ret == 0) {
      *priority = PRIORITY_LOW;
      ret = queue_.pop(job);
    } else {
      *priority = PRIORITY_HIGH;
    }
    if (ret != 0) {
      pthread_mutex_unlock(&mutex_);
      pthread_cond_broadcast(&not_full_);
      return ret;
    } else {
      if (shutdown_) {
        pthread_mutex_unlock(&mutex_);
        pthread_cond_broadcast(&not_empty_);
        return 0;
      } else {
        pthread_mutex_unlock(&mutex_);
      }
    }
    // fprintf(stderr, "again %d\n", a++);
  }
}

void ConsumerProducer::print_stats(void) {
  char buf[1024];
  get_stats_string(buf, sizeof(buf));
  inno_log_info("%s", buf);
  return;
}

void ConsumerProducer::get_stats_string(char *buf, size_t buf_size) {
  // xxx todo: this is not protected by mutex, so the number may off a little
  size_t active_time = 0;
  size_t len = 0;
  buf[0] = 0;
  for (int i = 0; i < PRIORITY_MAX; i++) {
    if (added_job_[i] == 0) {
      continue;
    }
    int ret = snprintf(buf + len, buf_size - len,
                       "%s queue#%d added=%" PRI_SIZEU
                       " finished=%" PRI_SIZEU
                       " dropped=%" PRI_SIZEU
                       " blocked=%" PRI_SIZEU
                       " wait=%" PRI_SIZEU
                       "us process=%" PRI_SIZEU
                       "us drop=%" PRI_SIZEU "us "
                       "pid=%d ",
                       name_, i,
                       added_job_[i], finished_job_[i], dropped_job_[i],
                       blocked_job_[i],
                       (finished_job_[i] + dropped_job_[i]) > 0 ?
                       total_wait_time_[i] /
                       (finished_job_[i]+ dropped_job_[i]) /
                       1000 :
                       0,
                       finished_job_[i] > 0 ?
                       total_process_time_[i] / finished_job_[i] / 1000 : 0,
                       total_drop_time_[i] > 0 ?
                       total_drop_time_[i] / dropped_job_[i] / 1000 : 0,
                       pid_);
    if (ret >= ssize_t(buf_size - len)) {
      buf[buf_size - 1] = 0;
      return;
    }
    len += ret;
    active_time += total_process_time_[i] + total_drop_time_[i];
  }

  size_t end_time = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
  size_t elapse_time = end_time - start_time_;

  if (elapse_time > 0 && active_time > 0) {
    size_t elapse_delta = elapse_time - last_elapse_time_;
    size_t active_delta = active_time - last_active_time_;
    double ratio = 0;
    if (elapse_delta > 0 && active_delta > 0) {
      ratio = active_delta * 100.0 / elapse_delta;
    }
    int ret = snprintf(buf + len, buf_size - len,
                       "elapsed_time=%" PRI_SIZEU "/%" PRI_SIZEU
                       "ms active_time=%" PRI_SIZEU
                       "/%" PRI_SIZEU "ms "
                       "ratio=%.2f%%/%.2f%%",
                       elapse_delta / 1000000,
                       (end_time - start_time_) / 1000000,
                       active_delta / 1000000,
                       active_time / 1000000,
                       ratio,
                       active_time * 100.0 / (end_time - start_time_));
    if (ret >= ssize_t(buf_size - len)) {
      buf[buf_size - 1] = 0;
      return;
    }
    if (elapse_delta > 0 && active_delta > 0) {
      last_elapse_time_ = elapse_time;
      last_active_time_ = active_time;
    }
  }
  return;
}

}  // namespace innovusion
