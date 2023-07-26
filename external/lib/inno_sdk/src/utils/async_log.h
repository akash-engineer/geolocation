/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 * @File Name: async_log.h
 * @brief :  This log module  will provide the async log.
 *           The caller can choose the mode of sync or  async
 * @Version : 1.0
 * @Creat Date : 2022-03-18
 */

#ifndef UTILS_ASYNC_LOG_H_
#define UTILS_ASYNC_LOG_H_

#include <limits.h>

#include <algorithm>
#include <mutex>  // NOLINT
#include <string>
#include <unordered_map>
#include <vector>
#include <set>

#include "utils/utils.h"
#include "utils/consumer_producer.h"
#include "utils/mem_pool_manager.h"
#include "utils/log.h"

namespace innovusion {
typedef struct {
  size_t  normal_ret;
  size_t  block_ret;
  size_t  giveUp_ret;
  size_t  dropHead_ret;
  size_t  error_ret;
} AsyncLogDebugInfo;

// cross include and earlier declare
class MemPool;

/**
 * @class : Async Job Thread Manager
 * This async thread will do the job for log and
 * packet deliver in callback module
 * Maybe will be used in other module.
 */
class AsyncLogManager {
 public:
  // Constructor
  AsyncLogManager(const char *name,
                  int priority,
                  ConsumeFunc consume_func,
                  void *consume_context,
                  int queue_size,
                  size_t bufNum = 50,
                  size_t bufSize = 1024*4);
  // Destroy
  virtual ~AsyncLogManager();
  // malloc the buff
  logContextInfo* alloc_buffer(size_t len);
  // delete the buff
  void free_buffer(logContextInfo *buffer_p);
  // Start the  async thread
  // add the job
  int add_log_job(const logContextInfo &log_info_p,
                  bool discardable);
  // wait the ACK
  // check the log  need call thread or not
  bool is_log_worker();
  // get debug info
  // AsyncLogDebugInfo* get_debug_info();

  void flush_and_pause() {
    cp_async_job_thread_->flush_and_pause();
  }

  void resume() {
    cp_async_job_thread_->resume();
  }

  // print stats
  void print_stats() {
    if (cp_async_job_thread_) {
      cp_async_job_thread_->print_stats();
    }
  }

#ifdef ASYCLOG_UNITEST_ENABLE
  // add this for debug
  MemPool * get_mempool() {
    return poolP_;
  }
#endif

  // shutdown
  void shutdown();
  // add the static process function to the consumer of async log
  static int process(void *job, void *ctx, bool prefer);
//
 private:
  // add the default value
  const int worker_num_ = 1;
  const int hi_cp_queue_size_ = 0;
  const size_t cpusetsize_ = 0;
  const int alignment_ = 32;
  // handle for job thread
  ConsumerProducer *cp_async_job_thread_;
  // Property
  // The buffer pool  number
  size_t async_job_bufnumber_;  // default 50
  // Single buffer size
  size_t async_job_bufsize_;  // default 4k
  // handle for buffer pool
  MemPool * poolP_;
  // worker id
  pthread_t * threads_p_;
  // default status is false
  bool async_running_ = false;
  // mutex
  // AsyncLogDebugInfo  debug_info_;
};

}  // namespace innovusion
#endif  // UTILS_ASYNC_LOG_H_
