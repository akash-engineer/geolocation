/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_CONFIG_H_
#define UTILS_CONFIG_H_

#include <limits.h>

#include <mutex>  // NOLINT
#include <string>
#include <unordered_map>
#include <vector>

#include "./log.h"

namespace innovusion {

class Config {
 public:
  Config();
  virtual ~Config();
  bool is_same_type(const std::string &n);
  int set_key_value(const std::string &cfg_key_in,
                    const std::string &cfg_value_in);
  virtual const char* get_type() const = 0;
  /* only copy when version are different */
  bool copy_from_src(Config *src);

 protected:
  /**
   * set a double value config
   * @param key
   * @param value
   * @return
   */
  virtual int set_key_value_(const std::string &key,
                             double value) = 0;
  /**
   * set a string value config
   * @param key
   * @param value
   * @return
   */
  virtual int set_key_value_(const std::string &key,
                             const std::string value) {
    // do nothing in base config
    return -1;
  }
  virtual void *get_start_() = 0;
  virtual size_t get_size_() = 0;

  inline uint64_t get_version_locked_() {
    // inno_log_assert(mutex_.owns_lock(), "not locked");
    return version_;
  }

  inline uint64_t set_version_locked_(uint64_t r) {
    // inno_log_assert(mutex_.owns_lock(), "not locked");
    version_ = r;
    return r;
  }

  void inc_version_();

 protected:
  std::mutex mutex_;

 private:
  uint64_t version_;
};

#define BEGIN_CFG_MEMBER()                      \
 private:                                       \
  void *get_start_() override {                  \
    return &start_;                             \
  }                                             \
  size_t get_size_() override {                           \
    return uintptr_t(&end_) - uintptr_t(get_start_());   \
  }                                                      \
  int32_t start_;                                        \
  public:

#define END_CFG_MEMBER()                        \
 private:                                       \
  int32_t end_;

#define SET_CFG(name)                           \
  do {                                          \
     if (strcmp(key.c_str(), #name) == 0) {     \
       std::unique_lock<std::mutex> lk(mutex_); \
       (name) = value;                            \
       return 0;                                \
     }                                          \
  } while (0)

class ConfigManager {
 public:
  explicit ConfigManager(const char *basename);
  ~ConfigManager();
  void add_config(Config *c);
  void remove_config(Config *c);
  void play_config();

  int set_config_key_value(const std::string &cfg_key_in,
                           const std::string &cfg_value_in,
                           bool from_app);

 protected:
  std::mutex mutex_;

 private:
  const char *basename_;
  std::unordered_map<std::string, std::vector<Config *>> configs_;
  std::unordered_map<std::string,
                     std::unordered_map<std::string, std::string>> history_;
};

/***********************
  Example Config class
 ***********************/
class ExampleConfig: public Config {
 public:
  ExampleConfig() : Config() {
    test1 = 0;  // <== ADD_MEMBER_STEP1: init the member
    test2 = 0;
    test3 = 0;
  }
  const char* get_type() const override {
    return "Example";  // <== different Config class must return different name
  }

  int set_key_value_(const std::string &key,
                             double value) override {
    SET_CFG(test1);  // <== ADD_MEMBER_STEP2
    SET_CFG(test2);
    SET_CFG(test3);
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  uint64_t test1;  // <== ADD_MEMBER_STEP3: declare type
  bool test2;
  double test3;
  END_CFG_MEMBER()
};

}  // namespace innovusion
#endif  // UTILS_CONFIG_H_
