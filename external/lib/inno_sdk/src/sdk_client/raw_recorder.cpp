/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <fcntl.h>

#ifndef __MINGW64__
#include <netinet/in.h>
#endif

#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <utility>
#include <vector>

#include "sdk_client/raw_recorder.h"
#include "sdk_client/lidar_client.h"
#include "sdk_common/inno_lidar_packet_utils.h"
#include "utils/utils.h"

// #include "raw_recorder.h"
namespace innovusion {
RawReceiver::RawReceiver(InnoLidarClient *l, int udp_port,
                         std::string save_path) {
  inno_log_verify(l, "lidar client is null!");
  this->lidar_client_ = l;
  this->udp_port = udp_port;
  this->save_path = std::move(save_path);
}

void *RawReceiver::receive_loop_() {
// set up udp
  struct timeval tv{};
  std::vector<InnoUdpOpt> opts;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;
  opts.emplace_back(SOL_SOCKET, SO_RCVTIMEO,
                    &tv, sizeof(tv),
                    "SO_RCVTIMEO");
  int fd = InnoUdpHelper::bind(udp_port, opts);
  if (fd < 0) {
    inno_log_error("create socket failed, port:%d", udp_port);
    return nullptr;
  }

  char buf[65536];
  while (!lidar_client_->it_raw_recorder_->has_shutdown()) {
    // recvfrom udp_port
    struct sockaddr_in serv_addr{};
    socklen_t len = sizeof(serv_addr);
    ssize_t n;
    while (-1 == (n = recvfrom(fd, buf, sizeof(buf),
                               MSG_WAITALL,
                               (struct sockaddr *)&serv_addr,
                               &len))
           && errno == EINTR) {
    }

//     inno_log_info("reading from udp:%d, errno:%d, port:%d",
//     n, errno, udp_port);

    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      } else {
        // should we signal when other error occurred ?
        inno_log_error_errno("recv error %d", udp_port);
      }
    } else {
      Raw4UdpHeader header{};
      inno_log_verify(n <= 65535, "n too big: %" PRI_SIZED "", n);
      if (InnoDataPacketUtils::raw4_header_from_net(buf, n, &header)) {
        if (msgid_saver_map_.find(header.idx)
            == msgid_saver_map_.end()) {
          // creat saver for new msg
          inno_log_info("create saver for msg %u", header.idx);
          auto *saver = new RawSaver(this->lidar_client_, this, header.idx);
          inno_log_verify(saver, "saver init failed for msg %u", header.idx);
          msgid_saver_map_.emplace(header.idx, saver);
          // clean saver map
          auto it = msgid_saver_map_.begin();
          while (it != msgid_saver_map_.end()) {
            if (it->second->destroied()) {
              delete it->second;
              it = msgid_saver_map_.erase(it);
            } else {
              it++;
            }
          }
          if (msgid_saver_map_.size() > kMaxMapSize) {
            inno_log_warning("Current saver count:%" PRI_SIZEU "",
                             msgid_saver_map_.size());
          }
        }
        msgid_saver_map_[header.idx]->add_data(buf, n);
      } else {
        // do nothing
      }
    }
  }
  // waiting all savers stopped
  for (auto it = msgid_saver_map_.begin(); it != msgid_saver_map_.end(); ++it) {
    inno_log_verify(it->second, "saver is impossible null here");
    it->second->shutdown();
    delete it->second;
    it->second = nullptr;
  }
  msgid_saver_map_.clear();
  inno_log_info("recorder thread shut down.");
  return nullptr;
}

void *RawReceiver::start(void *ctx) {
  inno_log_verify(ctx, "ctx of RawReceiver is not set");
  auto *rc = reinterpret_cast<RawReceiver *>(ctx);
  rc->receive_loop_();
  return nullptr;
}

RawSaver::RawSaver(InnoLidarClient *lidar_client,
                   RawReceiver *receiver, uint32_t msg_idx) {
  lidar_client_ = lidar_client;
  receiver_ = receiver;
  tmp_fd_ = -1;
  save_path_ = std::string(receiver->save_path);
  msg_idx_ = msg_idx;
  status_ = Status::WORKING;
  pthread_condattr_init(&condattr_);
  pthread_condattr_setclock(&condattr_, CLOCK_MONOTONIC);
  pthread_mutex_init(&mutex_status_, nullptr);
  pthread_mutex_init(&mutex_cache_, nullptr);
  pthread_cond_init(&cond_cache_, &condattr_);
  pthread_cond_init(&cond_status_, &condattr_);
  pthread_create(&thread_flush_, nullptr,
                 flush_thread_func, this);
  pthread_create(&thread_timer_, nullptr,
                 timer_start_, this);
}

RawSaver::~RawSaver() {
  shutdown();
  pthread_mutex_destroy(&mutex_cache_);
  pthread_mutex_destroy(&mutex_status_);
  pthread_cond_destroy(&cond_cache_);
  pthread_cond_destroy(&cond_status_);
  pthread_condattr_destroy(&condattr_);
  inno_log_verify(cache_.empty(),
                  "[sn%s-%s-%u] cache is not empty when saver exit!",
                  sn_.c_str(), cause_.c_str(), msg_idx_);
}

/**
 * cache data into cache_
 * call by receiver
 */
void RawSaver::add_data(char *buf, uint32_t len) {
  // status
  inno_log_verify(buf, "added buf is null!");
  inno_log_verify(len > Raw4UdpHeader::kHeaderSize, "bad data");
  pthread_mutex_lock(&mutex_status_);
  if (status_ != Status::WORKING) {
    pthread_mutex_unlock(&mutex_status_);
    return;
  }
  pthread_mutex_unlock(&mutex_status_);
  // signal timer if received any data
  pthread_cond_signal(&cond_status_);

  // data type
  Raw4UdpHeader header{};
  if (InnoDataPacketUtils::raw4_header_from_net(buf, len, &header)) {
    switch (header.field_type) {
      case InnoRaw4Packet::TYPE_SN:
        // sn
        sn_ = std::string(buf + Raw4UdpHeader::kHeaderSize,
                          len - Raw4UdpHeader::kHeaderSize);
        InnoUtils::remove_all_chars(&sn_, RawSaver::invalid_filename_chars);
        inno_log_info("got sn: %s", sn_.c_str());
        break;
      case InnoRaw4Packet::TYPE_CAUSE:
        // cause
        cause_ = std::string(buf + Raw4UdpHeader::kHeaderSize,
                             len - Raw4UdpHeader::kHeaderSize);
        InnoUtils::remove_all_chars(&cause_, RawSaver::invalid_filename_chars);
        inno_log_info("got cause: %s", cause_.c_str());
        break;
      // just copy raw data into cache
      case InnoRaw4Packet::TYPE_RAWDATA:
      {
        // if received id == expected id then cache and notify
        // < expected id then drop
        // > expected id then cache
        pthread_mutex_lock(&mutex_cache_);
        if (header.field_sequence_id >= expect_id_) {
          auto *data_buf = new RawDataBuf(buf, len);
          inno_log_verify(data_buf, "Create RawDataBuf failed!");
          cache_.emplace(header.field_sequence_id, data_buf);
        }
        if (header.field_sequence_id == expect_id_) {
          pthread_cond_signal(&cond_cache_);
        }
        pthread_mutex_unlock(&mutex_cache_);
        break;
      }

      default:
        inno_log_panic("unknown packet type:%d", header.field_type);
    }
  } else {
    inno_log_error("parse data error, drop it.");
  }
}

void *RawSaver::flush_thread_func(void *ctx) {
  auto *rs = reinterpret_cast<RawSaver *>(ctx);
  rs->flush_loop_();
  return nullptr;
}

/**
 *
 */
void RawSaver::flush_loop_() {
  // keep processing while working
  bool tmp_file_done = false;
  pthread_mutex_lock(&mutex_status_);
  while (status_ < Status::STOPPED) {
    pthread_mutex_unlock(&mutex_status_);
    // wait fill data into cache
    pthread_mutex_lock(&mutex_cache_);
    // if cache is empty or there isn't the expected sid, wait for new data
    // time out will also wake up
    if (cache_.find(expect_id_) == cache_.end()) {
      // add_data() will signal when expected id received
      // return ETIMEDOUT when wait expected id timeout
      timespec ts{};
      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_sec += kExpectIdTimeOutDefaultS;
      pthread_cond_timedwait(&cond_cache_, &mutex_cache_, &ts);
    }
    // timeout and no new data received, continue waiting
    if (cache_.empty()) {
      pthread_mutex_unlock(&mutex_cache_);
      pthread_mutex_lock(&mutex_status_);
      continue;
    }
    // expected id received, flush it
    if (cache_.find(expect_id_) != cache_.end()) {
      RawDataBuf *data_buf = cache_.find(expect_id_)->second;
      uint32_t eid = expect_id_;
      expect_id_++;
      pthread_mutex_unlock(&mutex_cache_);
      tmp_file_done = save_to_tmp_file_(data_buf);
      clear_cache_by_sid_(eid);
      if (tmp_file_done) {
        finish_();
      }
      // update expect id
      pthread_mutex_lock(&mutex_status_);
      continue;
    } else {
      // expected id still not received in time, find smallest as expected id
      size_t i = 0;
      while (cache_.find(expect_id_) == cache_.end() && i++ < cache_.size()) {
        expect_id_++;
      }
      pthread_mutex_unlock(&mutex_cache_);
      pthread_mutex_lock(&mutex_status_);
      continue;
    }
  }
  pthread_mutex_unlock(&mutex_status_);
  // close tmp file and rename
  if (!tmp_file_done) {
    finish_();
  }
  clear_cache_();
  inno_log_info("[sn%s-%s-%u] cache cleaned",
                sn_.c_str(), cause_.c_str(), msg_idx_);
  inno_log_info("[sn%s-%s-%u] flush thread exit",
                sn_.c_str(), cause_.c_str(), msg_idx_);
}

void RawSaver::clear_cache_() {
  inno_log_verify(status_ >= Status::STOPPED,
                  "[sn%s-%s-%u] must be stopped before clear cache",
                  sn_.c_str(), cause_.c_str(), msg_idx_);
  pthread_mutex_lock(&mutex_cache_);
  while (!cache_.empty()) {
    if (cache_.begin()->second) {
      delete cache_.begin()->second;
      cache_.begin()->second = nullptr;
      cache_.erase(cache_.begin());
    }
  }
  pthread_mutex_unlock(&mutex_cache_);
}

void RawSaver::clear_cache_by_sid_(uint32_t sid) {
  pthread_mutex_lock(&mutex_cache_);
  if (cache_.find(sid) != cache_.end()) {
    delete cache_.find(sid)->second;
    cache_.find(sid)->second = nullptr;
    cache_.erase(sid);
  }
  pthread_mutex_unlock(&mutex_cache_);
}

//  ./;'\"!@#$%^&*<>?:{}[]()|~` AND \n
const char *RawSaver::invalid_filename_chars = R"(" ./;'\"!@#$%^&*<>?:{}[]()|~`
                                                  ")";
void RawSaver::finish_() {
  char full_save_path[FILENAME_MAX + 1];
  int k = snprintf(full_save_path, FILENAME_MAX, "%s/sn%s-%s%s.inno_raw",
           save_path_.c_str(), sn_.c_str(), cause_.c_str(),
           incomplete_ ? "-incomplete" : "");
  inno_log_verify(k < FILENAME_MAX, "file name too long: %s", full_save_path);
  if (::access(full_save_path, F_OK) == 0) {
    inno_log_info("[sn%s-%s-%u] raw data already exist and will be replaced.",
                  sn_.c_str(), cause_.c_str(), msg_idx_);
  }
  ::rename(tmp_path_.c_str(), full_save_path);
  inno_log_info("rename file: %s->%s", tmp_path_.c_str(), full_save_path);
  ::remove(tmp_path_.c_str());
  // if total raw data file count  >= 3, remove the oldest
  total_size_control_();
  pthread_mutex_lock(&mutex_status_);
  if (status_ < Status::STOPPED) {
    status_ = Status::STOPPED;
    pthread_cond_signal(&cond_status_);
  }
  pthread_mutex_unlock(&mutex_status_);

  // send a message RAW_FILE_RECORDING_DONE
  lidar_client_->do_message_callback_fmt(
    INNO_MESSAGE_LEVEL_INFO,
    INNO_MESSAGE_CODE_RAW_RECORDING_FINISHED,
    "[sn%s-%s-%u] raw data recording finished",
    sn_.c_str(), cause_.c_str(), msg_idx_);
}

void RawSaver::total_size_control_() {
  DIR *dp = ::opendir(save_path_.c_str());
  struct dirent *entry;
  struct stat statbuf{};
  if (dp == nullptr) {
    return;
  }

  uint64_t total_size = 0;
  std::string oldest_raw_file;
  uint64_t oldest_raw_file_mtime = InnoUtils::get_time_ns(CLOCK_REALTIME);
  while ((entry = ::readdir(dp)) != nullptr) {
    if (InnoUtils::ends_with(entry->d_name, ".inno_raw")) {
      std::string entry_full_name = save_path_ + entry->d_name;
#ifndef __MINGW64__
      ::lstat(entry_full_name.c_str(), &statbuf);
      total_size += statbuf.st_size;
      uint64_t mtime = InnoUtils::get_timestamp_ns(statbuf.st_mtim);
#else
      ::stat(entry_full_name.c_str(), &statbuf);
      total_size += statbuf.st_size;
      uint64_t mtime = statbuf.st_mtime * 1000000000;
#endif
      if (mtime < oldest_raw_file_mtime) {
        oldest_raw_file = entry_full_name;
        oldest_raw_file_mtime = mtime;
      }
    }
  }
  ::closedir(dp);
  if (total_size > kMaxTotalRawFileSize) {
    inno_log_info("Total raw data size in path %s is %" PRI_SIZEU ","
                  " exceed the limit size %" PRI_SIZEU ", delete file %s",
                  save_path_.c_str(),
                  total_size,
                  kMaxTotalRawFileSize,
                  oldest_raw_file.c_str());
    ::remove(oldest_raw_file.c_str());
  }
}

bool RawSaver::save_to_tmp_file_(RawDataBuf *data_buf) {
  Raw4UdpHeader header{};
  inno_log_verify(data_buf && data_buf->is_valid(), "data_buf");
  if (!InnoDataPacketUtils::raw4_header_from_net(data_buf->start,
                                                data_buf->len, &header)) {
    inno_log_error("bad data: %p????, start:%p, len:%u",
                   data_buf, data_buf->start, data_buf->len);
    return false;
  }

  if (static_cast<int>(header.field_sequence_id) != last_flush_sid_ + 1) {
    incomplete_ = true;
  }
  last_flush_sid_ = static_cast<int>(header.field_sequence_id);

  if (tmp_fd_ < 0) {
    tmp_path_ = save_path_ + "/" + std::to_string(msg_idx_) + "-tmp.inno_raw."
        + std::to_string(InnoUtils::get_time_ns(CLOCK_MONOTONIC));
    tmp_fd_ = InnoUtils::open_file(tmp_path_.c_str(),
                                   O_WRONLY | O_CREAT | O_TRUNC, 0777);
    inno_log_verify(tmp_fd_ >= 0, "open %s failed.", tmp_path_.c_str());
  }

  ssize_t written;
  while ( -1 == (written = ::write(tmp_fd_,
          data_buf->start + Raw4UdpHeader::kHeaderSize,
          data_buf->len - Raw4UdpHeader::kHeaderSize)) &&
          (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
  }

  // finish recording when reach the end or error occurred
  if (header.is_field_end() || written < 0) {
    if (written < 0) {
      inno_log_error("[sn%s-%s-%u] write to file failed."
                     " stop recording. errno:%d",
                     sn_.c_str(), cause_.c_str(), msg_idx_, errno);
    }
    int ret = InnoUtils::close_fd(tmp_fd_);
    if (ret != 0) {
      inno_log_error_errno("[sn%s-%s-%u] close tmp file failed.",
                           sn_.c_str(), cause_.c_str(), msg_idx_);
    }
    tmp_fd_ = -1;
    return true;
  }
  return false;
}

void *RawSaver::timer_start_(void *ctx) {
  auto *rs = reinterpret_cast<RawSaver *>(ctx);
  rs->timer_loop_();
  return nullptr;
}

void RawSaver::timer_loop_() {
  while (true) {
    struct timespec ts{};
    pthread_mutex_lock(&mutex_status_);
    if (status_ == Status::WORKING) {
      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_sec += kDataStreamingStopTimeOutDefaultS;
      int ret = pthread_cond_timedwait(&cond_status_, &mutex_status_, &ts);
      if (ret == ETIMEDOUT) {
        // timeout: streaming stop
        status_ = Status::STOPPED;
        inno_log_info("[sn%s-%s-%u] data streaming stopped",
                      sn_.c_str(), cause_.c_str(), msg_idx_);
      }  // else continue waiting or be destroyed
      pthread_mutex_unlock(&mutex_status_);
    } else if (status_ == Status::STOPPED) {
      inno_log_info("[sn%s-%s-%u] saver stopped",
                    sn_.c_str(), cause_.c_str(), msg_idx_);
      // after stopped for kDestroyTimeOutDefaultS, enter destroyed status
      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_sec += kDestroyTimeOutDefaultS;
      int ret = pthread_cond_timedwait(&cond_status_, &mutex_status_, &ts);
      if (ret == ETIMEDOUT) {
        status_ = Status::DESTROYED;
      } else {
        inno_log_verify(status_ == Status::DESTROYED,
                        "[sn%s-%s-%u] error status",
                        sn_.c_str(), cause_.c_str(), msg_idx_);
        inno_log_info("[sn%s-%s-%u] saver destroyed by command",
                      sn_.c_str(), cause_.c_str(), msg_idx_);
      }
      pthread_mutex_unlock(&mutex_status_);
    } else if (status_ == Status::DESTROYED) {
      pthread_mutex_unlock(&mutex_status_);
      inno_log_info("[sn%s-%s-%u] saver destroyed",
                    sn_.c_str(), cause_.c_str(), msg_idx_);
      break;
    } else {
      // Unknown Status
      inno_log_panic("[sn%s-%s-%u] error status",
                     sn_.c_str(), cause_.c_str(), msg_idx_);
    }
  }
  pthread_mutex_unlock(&mutex_status_);
  pthread_cond_signal(&cond_cache_);
  void *r;
  pthread_join(thread_flush_, &r);
}

void RawSaver::shutdown() {
  // shutdown in working status or in stopped
  inno_log_info("[sn%s-%s-%u] shutting down",
                sn_.c_str(), cause_.c_str(), msg_idx_);
  pthread_mutex_lock(&mutex_status_);
  void *ret;
  if (status_ == Status::DESTROYED) {
    pthread_mutex_unlock(&mutex_status_);
    // saver has destroyed itself cause timeout
    pthread_join(thread_timer_, &ret);
  } else {
    status_ = Status::DESTROYED;
    pthread_mutex_unlock(&mutex_status_);
    pthread_cond_signal(&cond_status_);
    pthread_join(thread_timer_, &ret);
  }
}

}  // namespace innovusion

