/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 *
 * Manager usage of vm mapped to /dev/mem 0x1000000~0x1dfffff
 * When create a MemPool, it's pool_ should be alloc use this class.
 * It will first alloc firstly try to alloc from memory map to /dev/mem.
 * If there is not enough memory in /dev/mem 0x1000000~0x1dfffff, it
 * will try to call calloc() func of stdlib to alloc memory.
 */

#ifndef UTILS_MEM_ALLOCATOR_H_
#define UTILS_MEM_ALLOCATOR_H_

#include <unistd.h>
#include <mutex>  // NOLINT

#include "./log.h"

#define MEM_ALLOC_DEFAULT 1
#define MEM_ALLOC_MAP 1
#ifndef ARCH_ARM64
#undef MEM_ALLOC_MAP
#define MEM_ALLOC_MAP 0
#endif

namespace innovusion {
class MemAllocDelegate;
class MemRange;

//=====================================================================
// MemAllocator
// Define interfaces.
//=====================================================================
class MemAllocator {
  friend MemAllocDelegate;
 public:
  virtual ~MemAllocator() = default;
 protected:
  virtual void *calloc(size_t nmemb, size_t size) = 0;
  virtual void free(void *buffer) = 0;
  virtual bool addr_is_valid(void *addr) const = 0;
  /*whether allocator instance is valid*/
  virtual bool is_valid() const = 0;
  virtual const char *get_name() const = 0;

 public:
  MemAllocator *next{};
};

//=====================================================================
// MemMapMemAllocator
// Use MemMap to map /dev/mem 0x1000000-0x1dffefff into virtual memory.
// Use a linked table to store ranges in this mapping space. A range
// may be free or in-use.
//=====================================================================

class MemMapMemAllocator final: MemAllocator {
  friend MemAllocDelegate;

 protected:
  void *calloc(size_t nmemb, size_t size) override;
  void free(void *buffer) override;
  bool addr_is_valid(void *addr) const override;
  bool is_valid() const override;
  const char *get_name() const override {
    return "MemMapMemAllocator";
  }
  MemMapMemAllocator();
  MemMapMemAllocator(size_t base_ddr, size_t high_addr);
  ~MemMapMemAllocator();

 private:
  MemRange *search_first_applicable_range_(size_t size_need);
  void *alloc_in_free_range_(MemRange *range, size_t size_need);
  MemRange *find_target_range_(void *buffer);
  void merge_contiguous_range_(MemRange *first, MemRange *second);

 private:
  MemRange *range_list_;
  static std::mutex mutex_;
  int mem_fd_;
  void *map_base_;
#ifndef __MINGW64__
  size_t page_size_ = sysconf(_SC_PAGESIZE);
#else
  size_t page_size_ = 4 * 1024;
#endif
  size_t base_addr_;
  size_t high_addr_;
};

class MemRange {
  friend MemMapMemAllocator;
 protected:
  MemRange(void *offset, size_t size):offset(offset), size(size) {
    end = reinterpret_cast<void *>(reinterpret_cast<char *>(offset) + size - 1);
    is_dirty = false;
    next = nullptr;
    pre = nullptr;
  }
  ~MemRange() = default;

 protected:
  void *offset;
  void *end;
  size_t size;
  bool is_dirty;
  MemRange *next;
  MemRange *pre;
};


//=====================================================================
// DefaultMemAllocator
// use std::alloc() and std::free()
//=====================================================================
class DefaultMemAllocator final: MemAllocator {
  friend MemAllocDelegate;

 protected:
  DefaultMemAllocator();
  void *calloc(size_t nmemb, size_t size) override;
  void free(void *buffer) override;
  bool addr_is_valid(void *addr) const override {
    return true;
  };
  bool is_valid() const override {
    return true;
  }
  const char *get_name() const override {
    return "DefaultMemAllocator";
  }

 private:
  static std::mutex mutex_;
};

class MemAllocDelegate {
 public:
  void *calloc(size_t nmemb, size_t size);
  void free(void *buffer);
  static MemAllocDelegate *get_instance();
  /**
   * return which allocator be used to allocated the buffer
   * @return allocator's name
   */
  const char *get_allocator_name(void *buffer);
  void setup_live_direct_memory_mode(size_t base_addr, size_t high_addr);

 private:
  MemAllocDelegate();
  ~MemAllocDelegate() {
    while (allocator_list_) {
      MemAllocator *head = allocator_list_;
      allocator_list_ = head->next;
      ::free(head);
    }
    allocator_list_ = nullptr;
  }
  static MemAllocDelegate *instance_;
  static std::mutex mutex_;
  MemAllocator *allocator_list_;
  std::mutex instance_mutex_;
  bool is_live_direct_memory_mode;
  uint32_t allocation_cnt_;
};
}  // namespace innovusion
#endif  // UTILS_MEM_ALLOCATOR_H_
