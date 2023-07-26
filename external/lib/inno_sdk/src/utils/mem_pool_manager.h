/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_MEM_POOL_MANAGER_H_
#define UTILS_MEM_POOL_MANAGER_H_

#include <limits.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdint.h>
#include <cstddef>
#include <map>
#include <string>

#include "utils/log.h"
#include "utils/mem_allocator.h"

namespace innovusion {
class MemPoolManager {
 public:
  MemPoolManager(const char *name,
                 void *buffer,
                 unsigned int unit_size,
                 unsigned int unit_number);
  ~MemPoolManager();
  void *alloc(unsigned int size);
  void free(void *buffer);
  const void *get_pool() const {
    return reinterpret_cast<const void *>(pool_);
  }
  bool is_manager_of(void *b) const {
    return b >= pool_ &&
        b < reinterpret_cast<char*>(pool_) + unit_size_ * unit_count_;
  }

 private:
  char *name_;
  void *pool_;
  unsigned int unit_size_;
  unsigned int unit_count_;
  unsigned int free_count_;
  uint64_t alloc_call_count_;
  uint64_t return_null_count_;
  uint64_t request_too_big_;
  pthread_mutex_t mutex_;
  bool * in_use_;
  int *next_free_;
  int first_free_;
  int last_free_;
};

class MemPool {
 public:
  // support sys malloc
  MemPool(const char *name,
          unsigned int unit_sz,
          unsigned int unit_nm,
          uint64_t alignment, bool is_sys_malloc = false);

  ~MemPool();
  void *alloc();
  void free(void *buffer);

  /**
   * @brief : is_manager_of
   * @param  address_p  :   Address Pointer
   * @return true  : Is pool pointer
   * @return false; : Not pool pointer
   */
  bool is_manager_of(void *address_p) const {
    return manager_->is_manager_of(address_p);
  }

 private:
  char *name_;
  void *pool_;
  void *aligned_pool_;
  unsigned int unit_size_;
  unsigned int unit_count_;
  uint64_t alignment_;
  MemPoolManager *manager_;
  MemAllocDelegate *alloc_delegate_;
  // support the sys malloc
  bool is_sys_malloc_ = false;
};

template <class T>
class ObjectPool {
 public:
  ObjectPool(const char *name,
             unsigned int unit_number,
             uint64_t alignment) :
      pool_(name,
            sizeof(T),
            unit_number,
            alignment) {
  }

  ~ObjectPool() {
  }

  virtual T *alloc() {
    return new (pool_.alloc()) T;
  }

  void free(T *o) {
    return pool_.free(o);
  }

 private:
  MemPool pool_;
};

}  // namespace innovusion
#endif  // UTILS_MEM_POOL_MANAGER_H_
