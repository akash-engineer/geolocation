/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <pcap.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <map>
#include "src/sdk_common/inno_lidar_api.h"
#include "src/sdk_common/inno_lidar_packet_utils.h"
#include "src/utils/inno_lidar_log.h"


#define   PORT        8010
#define   RAW_PORT    8020

static const double kUsInSecond = 1000000.0;
static const double k10UsInSecond = 100000.0;
static const uint32_t kInnoMaxListSize = 10;

#pragma pack(push)
#pragma pack(1)

typedef struct  {
  int record_pcd;
}ParseConfig;

typedef struct {
  char data[1024 * 10];
} CacheData;

typedef struct {
  std::string ip_path;
  std::string pcd_path;
  int state_fd;
  int message_fd;
  std::list<CacheData> *cache_data;
}DispatcherInfo;

typedef struct {
  char name[8];           // "lidarDt"
  char file_version[8];   //  "01.00"
} DataInfo;

typedef struct {
  uint32_t recv_addr;
  uint16_t rece_port;
} SocketDect;

struct LidarHeader {
  DataInfo info;
  SocketDect dest;
  uint8_t padding[106];
};

struct TranscationHeader {
  uint64_t send_timestamp;
  uint64_t recv_timestamp;
  uint64_t size;
};

#pragma pack(pop)

/***********************
 * class PcdRecorder
 ***********************/
class PcdRecorder {
 public:
  static const size_t kMaxBufferSize = 256 * 1024;
  static const size_t kHeaderSize = 360;

 public:
  explicit PcdRecorder(std::string folder,
                       const uint64_t frame_id,
                       double timestamp_sec,
                       bool reflectance)
      : frame_id_(frame_id)
      , fd_(-1)
      , point_count_(0)
      , reflectance_(reflectance)
      , buffer_cursor_(0) {
    filename_ = folder + "/INNO_IDX_" + \
                            std::to_string(frame_id_)\
                            + "_TIME_" + std::to_string(timestamp_sec)\
                            + ".pcd";
    fd_ = open(filename_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    inno_log_verify(fd_ >= 0, "cannot open %s", filename_.c_str());
#ifdef DEBUG
    inno_log_info("open pcd file %s to record frame %lu",
                  filename_.c_str(), frame_id);
#endif
    write_dummy_header_();
    return;
  }

  ~PcdRecorder() {
    close_();
    if (fd_ >= 0) {
      close(fd_);
    }
    fd_ = -1;
    return;
  }

  void add_points(const uint64_t frame_id,
                  const double x, const double y, const double z,
                  const uint32_t ref, const uint32_t channel,
                  const uint32_t in_roi,
                  const uint32_t facet,
                  const uint32_t m_ret,
                  const uint32_t confidence_level,
                  const uint32_t flags,
                  const uint32_t elongation,
                  const double timestamp_sec,
                  const uint32_t scanline,
                  const uint32_t scan_idx) {
    if (fd_ < 0) {
      return;
    }
    if (buffer_cursor_ + 100 > kMaxBufferSize) {
      // inno_log_error("skip point because buffer is too small");
      // return;
      flush_buffer_();
    }
    int wri = snprintf(buffer_ + buffer_cursor_,
                       sizeof(buffer_) - buffer_cursor_,
                       "%.3f %.3f %.3f %u %u "
                       "%u %u %u %u %u %u "
                       "%.5f %u %u %lu\n",
                       x, y, z, ref, channel,
                       in_roi, facet, m_ret, confidence_level,
                       flags, elongation,
                       timestamp_sec, scanline, scan_idx, frame_id);
    buffer_cursor_ += wri;
    inno_log_verify(buffer_cursor_ < sizeof(buffer_),
                    "not enough buffer");
    point_count_++;
    return;
  }

 private:
  void write_dummy_header_() {
    memset(header_buffer_, ' ', sizeof(header_buffer_));
    header_buffer_[sizeof(header_buffer_) - 1] = '\n';
    int r = write(fd_, header_buffer_, sizeof(header_buffer_));
    inno_log_verify(r >= 0, "cannot write header %s", filename_.c_str());
  }
  void rewrite_header_() {
    const char *pcd_header =
        "# .PCD v.7 - Point Cloud Data file format\n"
        "FIELDS "
        "x y z %s channel roi facet multi_return confid_level "
        "flag elongation timestamp "
        "scanline scan_idx frame_id\n"
        "SIZE 4 4 4 4 4 4 4 4 4 4 4 8 4 4 4\n"
        "TYPE F F F U U U U U U U U F U U U\n"
        "COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n"
        "WIDTH %lu\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %lu\n"
        "DATA ascii";
    int r;
    r = snprintf(header_buffer_, sizeof(header_buffer_),
                 pcd_header, reflectance_ ? "reflectance" : "intensity",
                 point_count_, point_count_);
    inno_log_verify(r < int32_t(sizeof(header_buffer_)),
                    "buffer too small %d vs %lu", r, sizeof(header_buffer_));
    inno_log_info("write %lu points to pcd file %s",
                  point_count_,
                  filename_.c_str());
    FILE *file = fopen(filename_.c_str(), "r+");
    inno_log_verify(file != NULL, "Unable to open file %s", filename_.c_str());

    fseek(file, 0, SEEK_SET);
    size_t fr = fwrite(header_buffer_, 1, strlen(header_buffer_), file);
    inno_log_verify(fr == strlen(header_buffer_),
                    "fwrite return %lu", fr);
    fclose(file);
    return;
  }
  void flush_buffer_() {
    int r = write(fd_, buffer_, buffer_cursor_);
    inno_log_verify(r >= 0, "cannot write data to %s", filename_.c_str());
    buffer_cursor_ = 0;
  }

  void close_() {
    if (fd_ >= 0) {
      flush_buffer_();
      close(fd_);
      fd_ = -1;
      rewrite_header_();
    }
    return;
  }

 private:
  std::string filename_;
  uint64_t frame_id_;
  int fd_;
  uint64_t point_count_;
  bool reflectance_;
  uint64_t buffer_cursor_;
  char buffer_[kMaxBufferSize];
  char header_buffer_[kHeaderSize];
};



/***********************
 * class RawDataMessage
 ***********************/

//
//
//
class RawDataFile {
 private:
  uint8_t field_type_;

  int fd_{-1};
  std::string file_path_;

  using cache_item = std::pair<uint32_t, std::string>;
  using cache_vector = std::vector<std::pair<uint32_t, std::string>>;
  using cache_map = std::unordered_map<uint32_t, std::string>;
  cache_map cache_;

  bool has_finished_{false};
  uint32_t next_sequence_id_{0};

  uint32_t stat_field_;

 public:
  bool create_file(const std::string &file_path) {
    this->file_path_ = file_path;

    fd_ = ::open(file_path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    inno_log_verify(fd_ >= 0, "cannot create file, error: %d, file paht: %s",
                    errno, file_path_.c_str());

    inno_log_info("create raw file, file name: %s", file_path_.c_str());
    return true;
  }

  void save(const Raw4UdpHeader &header, const char *data, int len) {
    if (header.field_sequence_id == next_sequence_id_) {
      save_file_(header, data, len);
      pop_cache_();
    } else {
      // cache out-of-order fragment
      cache_.emplace(header.field_sequence_id, std::string(data, len));
    }
  }

  bool has_finished() { return has_finished_; }

  void close() {
    bool is_incomplete = !has_finished_;

    if (is_incomplete) {
      inno_log_info("raw dump cache : %s", file_path_.c_str());
      dump_cache_();
    }

    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }

    if (is_incomplete) {
      std::string new_path = file_path_ + ".incomplete";
      ::rename(file_path_.c_str(), new_path.c_str());
    }
  }

 private:
  void save_file_(const Raw4UdpHeader &header, const char *data, int len) {
    const char *fragment = data + Raw4UdpHeader::kHeaderSize;
    int fragment_size = len - Raw4UdpHeader::kHeaderSize;

    int r = ::write(fd_, fragment, fragment_size);
    inno_log_verify(r >= 0, "cannot write header, error: %d", errno);

    switch_next_(header.is_field_end());
  }

  //
  void switch_next_(bool is_flag_end) {
    next_sequence_id_++;
    if (is_flag_end) {
      stat_field_ = this->next_sequence_id_;
      has_finished_ = true;
    }
  }

  //
  void pop_cache_() {
    bool exist = true;

    while (exist && !this->has_finished_) {
      exist = cache_.find(this->next_sequence_id_) != cache_.end();
      if (exist) {
        const char *data = cache_[this->next_sequence_id_].data();
        size_t len = cache_[this->next_sequence_id_].size();

        Raw4UdpHeader header;
        InnoDataPacketUtils::raw4_header_from_net(data, len, &header);

        save_file_(header, data, len);
        cache_.erase(this->next_sequence_id_);
      }
    }
  }

  //
  void dump_cache_() {
    inno_log_warning("%s dump cache, next packet: %d, cache size: %lu.",
                     file_path_.c_str(), this->next_sequence_id_,
                     cache_.size());

    if (cache_.empty()) {
      return;
    }

    cache_vector tmp;
    tmp.insert(tmp.end(), cache_.begin(), cache_.end());

    std::sort(tmp.begin(), tmp.end(),
              [](const cache_item &lhs, const cache_item &rhs) {
                return lhs.first < rhs.first;
              });

    for (auto &item : tmp) {
      const char *data = item.second.data();
      size_t len = item.second.size();

      Raw4UdpHeader header;
      InnoDataPacketUtils::raw4_header_from_net(data, len, &header);

      // inno_log_info("%s dump: %d.", file_path_.c_str(), item.first);
      save_file_(header, data, len);
    }
  }
};

//
//
//
class RawDataMessage {
 public:
  explicit RawDataMessage(const std::string &folder, uint32_t msg_idx) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    std::string file_path = folder + "/inno_raw_" + std::to_string(msg_idx) +
                            "_" + std::to_string(ts.tv_sec);

    cause_field_.create_file(file_path + ".txt");
    rawdate_field_.create_file(file_path + ".raw");
  }

  //
  void save(const Raw4UdpHeader &header, const char *data, int len) {
    // sn field is short.
    if (header.field_type == InnoRaw4Packet::TYPE_SN) {
      sn_.append(data, len);
      return;
    }

    // order fragment: write file
    switch (header.field_type) {
      case InnoRaw4Packet::TYPE_CAUSE:
        cause_field_.save(header, data, len);
        break;

      case InnoRaw4Packet::TYPE_RAWDATA:
        rawdate_field_.save(header, data, len);
        break;

      default:
        break;
    }
  }

  //
  bool has_finished() {
    return cause_field_.has_finished() && rawdate_field_.has_finished();
  }

  //
  void close() {
    cause_field_.close();
    rawdate_field_.close();
  }

 private:
  std::string sn_;

  RawDataFile cause_field_;
  RawDataFile rawdate_field_;
};

/***********************
 * class PcapProcessor
 ***********************/
class PcapProcessor {
 public:
  static const uint32_t kMaxIPPackageSize = 1500;
  static const size_t kMaxBlockSize = 200 * 1024;

 public:
  PcapProcessor(const std::string &pcap_filename,
                const std::string &output_foldername,
                const ParseConfig &config)
      : filename_(pcap_filename)
      , output_foldername_(output_foldername)
      , current_frame_(-1)
      , frame_so_far_(-1)
      , pcd_recorder_(NULL) {
    current_frame_ = 0;
    write_data_df_len_ = 0;
    message_counter = 0;
    error_message_counter = 0;
    status_counter = 0;
    error_status_counter = 0;
    total_receive_counter = 0;
    total_successfully_counter = 0;
    total_failed_counter = 0;
    partial_sub_frame_counter = 0;
    data_counter = 0;
    error_data_counter = 0;
    current_cache_ = NULL;
    config_ = config;
  }

  ~PcapProcessor() {
    flush_cache_parse_data_();
    if (pcd_recorder_) {
      delete pcd_recorder_;
      pcd_recorder_ = NULL;
    }
    for (auto &fd : raw_fd_map_) {
      fd.second->close();   // dump incomplete raw file.

      delete fd.second;
      fd.second = nullptr;
    }
    raw_fd_map_.clear();

    inno_log_info("----------Summary----------\n"
      "data_counter = %lu, error_data_counter = %lu\n"
      "frame_counter = %lu, miss_frame_counter = %lu, "
      "partial_sub_frame_counter = %lu, miss_sub_frame_gap_counter = %lu\n"
      "miss_sub_frame_last_one_counter = %lu, "
      "miss_sub_frame_except_last_one_counter = %lu\n"
      "empty_sub_frame_counter = %lu\n"
      "message_counter = %lu, error_message_counter = %lu\n"
      "status_counter = %lu, error_status_counter = %lu\n"
      "total_receive_counter = %lu\n"
      "total_successfully_counter = %lu,"
      "total_failed_counter = %lu\n",
      data_counter, error_data_counter,
      summary_package_.get_frame_count(),
      summary_package_.get_miss_frame_count(),
      partial_sub_frame_counter,
      summary_package_.get_miss_sub_frame_gap_count(),
      summary_package_.get_miss_sub_frame_last_one_count(),
      summary_package_.get_empty_sub_frame_except_last_one_count(),
      summary_package_.get_empty_sub_frame_count(),
      message_counter, error_message_counter,
      status_counter, error_status_counter,
      total_receive_counter,
      total_successfully_counter, total_failed_counter);
  }

  void parse_inno_package(uint16_t destport, char *data, int len);
  static void process_pcap(u_char *user,
                  const struct pcap_pkthdr *pkthdr,
                  const u_char *packet);
  static void process_udp(u_char *arg, const struct pcap_pkthdr *pcap_pkt,
                               const u_char *packet, uint32_t saddr);

 protected:
  void process_data_cpoint_(const InnoDataPacket &pkt);
  bool parse_status_(char *data, int len);
  bool parse_data_(char *data, int len);
  bool parse_message_(char *data, int len);
  bool parse_raw_(char *data, int len);
  void counter_total_(bool ret);
  bool check_offset_(uint16_t expect_off, uint16_t offset);
  int set_current_ip_dispatcher(uint32_t saddr);
  void open_current_statue_msg_filename();
  void flush_cache_parse_data_();
  int create_folder_(const char* path);
  InnoDataPacket* get_and_cache_data_(const InnoDataPacket &pkt);

 private:
  inline void add_point_to_pcd_recorder_(const double ts_start_us,
                                         const int64_t frame_id,
                                         const double x, const double y,
                                         const double z,
                                         const uint32_t ref,
                                         const uint32_t channel,
                                         const uint32_t in_roi,
                                         const uint32_t facet,
                                         const uint32_t m_ret,
                                         const uint32_t confidence_level,
                                         const uint32_t flags,
                                         const uint32_t elongation,
                                         const double timestamp_sec,
                                         const uint32_t scanline,
                                         const uint32_t scan_idx,
                                         const bool reflectance) {
    // how many frames we have seen so far?
    // we want to skip the first frame, which may be partial
    if ((config_.record_pcd) &&
       (frame_so_far_ == -1 || current_frame_ != frame_id)) {
      frame_so_far_++;
      if (frame_so_far_ > 0) {
        if (pcd_recorder_) {
          delete pcd_recorder_;
          pcd_recorder_ = NULL;
        }
        pcd_recorder_ = new PcdRecorder(current_pcd_folder_,
                                        frame_id,
                                        timestamp_sec,
                                        reflectance);
        inno_log_verify(pcd_recorder_ != NULL, "cannot create pcd recorder");
      }
      current_frame_ = frame_id;
    }
    if (pcd_recorder_) {
      pcd_recorder_->add_points(frame_id, x, y, z,
                                  ref, channel,
                                  in_roi, facet, m_ret, confidence_level,
                                  flags, elongation,
                                  timestamp_sec,
                                  scanline, scan_idx);
    }
    return;
  }

 protected:
  std::string filename_;
  std::string output_foldername_;
  std::unordered_map<int, DispatcherInfo> map_dispather_info_;
  std::string current_ip_folder_;
  std::string current_pcd_folder_;
  ParseConfig config_;
  int current_status_fd_;
  int current_msg_fd_;
  std::list<CacheData> *current_cache_;
  int64_t current_frame_;
  int64_t frame_so_far_;
  PcdRecorder *pcd_recorder_;
  InnoSummaryPackage summary_package_;
  uint16_t write_data_df_len_;
  char data_df_buffer_[kMaxIPPackageSize];
  char msg_buffer_[kMaxIPPackageSize];
  char status_buffer_[kMaxIPPackageSize];
  uint64_t message_counter;
  uint64_t error_message_counter;
  uint64_t status_counter;
  uint64_t error_status_counter;
  uint64_t total_receive_counter;
  uint64_t total_successfully_counter;
  uint64_t total_failed_counter;
  uint64_t partial_sub_frame_counter;
  uint64_t data_counter;
  uint64_t error_data_counter;

  std::unordered_map<uint32_t, RawDataMessage*> raw_fd_map_;
};

/*
  this function shows how to enumerate blocks and points in the packet
  directly, and use InnoDataPacketUtils::get_xyzr_meter to get each
  point's x,y,z coodindate, then add it to the pcd recorder
*/
void PcapProcessor::process_data_cpoint_(const InnoDataPacket &pkt) {
  uint32_t return_number;
  uint32_t unit_size;
  // summary date package frame and sub frame
  summary_package_.summary_data_package(pkt);
  InnoDataPacketUtils::get_block_size_and_number_return(pkt,
                                                        &unit_size,
                                                        &return_number);
  double frame_timestamp_sec = pkt.common.ts_start_us / kUsInSecond;
  const InnoBlock *block =
      reinterpret_cast<const InnoBlock *>(&pkt.inno_block1s[0]);
  for (uint32_t i = 0; i < pkt.item_number;
        i++, block = reinterpret_cast<const InnoBlock *>
                    (reinterpret_cast<const char *>(block) + unit_size)) {
    // calculate (x,y,z) cartesian coordinate from spherical coordinate
    // for each point in the block
    // 1. use get_full_angles() to restore angle for each channel
    // 2. use get_xyzr_meter() to calculate (x,y,z)
    InnoBlockFullAngles full_angles;
    InnoDataPacketUtils::get_block_full_angles(&full_angles, block->header);
    for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
      for (uint32_t m = 0; m < return_number; m++) {
        const InnoChannelPoint &pt =
            block->points[InnoBlock2::get_idx(channel, m)];
        InnoXyzrD xyzr;
        if (pt.radius > 0) {
          InnoDataPacketUtils::get_xyzr_meter(
              full_angles.angles[channel],
              pt.radius, channel,
              &xyzr);
          add_point_to_pcd_recorder_(pkt.common.ts_start_us,
                                      pkt.idx, xyzr.x, xyzr.y, xyzr.z,
                                      pt.refl, channel,
                                      block->header.in_roi,
                                      block->header.facet,
                                      m,
                                      pkt.confidence_level,
                                      pt.type, pt.elongation,
                                      frame_timestamp_sec +
                                      block->header.ts_10us / k10UsInSecond,
                                      block->header.scan_id,
                                      block->header.scan_idx,
                                      pkt.use_reflectance);
        }
      }
    }
  }
  return;
}

bool PcapProcessor::parse_status_(char *data, int len) {
  InnoStatusPacket *pkt = reinterpret_cast<InnoStatusPacket *>(data);
  status_counter++;
  inno_log_trace("parse_status status_counter = %lu\n", status_counter);
  // sanity check after relase-2.0.0-rc138
  if (!InnoDataPacketUtils::check_status_packet(*pkt, 0)) {
    inno_log_error("corrupted pkt->idx = %lu", pkt->idx);
    error_status_counter++;
    return false;
  }

  int write_len = InnoDataPacketUtils::printf_status_packet(
      *pkt, status_buffer_, sizeof(status_buffer_));
  if (write_len > 0) {
    int r = write(current_status_fd_, status_buffer_, write_len);
    if (r < 0) {
      inno_log_warning("cannot write data to status fd %d.",
                                       current_status_fd_);
    }
  }

  return true;
}

bool PcapProcessor::parse_data_(char *data, int len) {
  InnoDataPacket *innopkt = NULL;
  InnoDataPacket *pkt = reinterpret_cast<InnoDataPacket *>(data);
  data_counter++;
  inno_log_trace("parse_data data_counter = %lu\n", data_counter);
  // sanity check after relase-2.0.0-rc138
  if (!InnoDataPacketUtils::check_data_packet(*pkt, 0)) {
    inno_log_error("check data error pkt->idx = %lu,  "
                  "miss sub frame due to lost IP segment= %u",
                  pkt->idx, pkt->sub_idx);
    error_data_counter++;
    return false;
  }

  innopkt = get_and_cache_data_(*pkt);
  if (innopkt == NULL) {
    return true;
  }
  process_data_cpoint_(*innopkt);
  if (current_cache_->size() > kInnoMaxListSize) {
    current_cache_->pop_back();
  }

  return true;
}


InnoDataPacket * PcapProcessor::get_and_cache_data_
                        (const InnoDataPacket &pkt) {
  CacheData tmpCache;
  InnoDataPacket *cache_package;

  memcpy(tmpCache.data, &pkt, sizeof(CacheData));
  if (current_cache_->size() < 1) {
    current_cache_->push_front(tmpCache);
    return NULL;
  } else {
    // insert the pkt
    std::list<CacheData>::iterator itr;
    for (itr = current_cache_->begin(); itr != current_cache_->end(); itr++) {
      cache_package = reinterpret_cast<InnoDataPacket *>((*itr).data);
      if (cache_package->idx < pkt.idx ||
            (cache_package->idx == pkt.idx &&
             cache_package->sub_idx < pkt.sub_idx)) {
        current_cache_->insert(itr, tmpCache);
        break;
      }
    }
    if (itr == current_cache_->end()) {
      current_cache_->push_back(tmpCache);
    }
    if (current_cache_->size() > kInnoMaxListSize) {
      cache_package = reinterpret_cast<InnoDataPacket *>
                      (current_cache_->back().data);
      inno_log_trace("cachepkg.idx= %ld, cachepkg.sub_idx = %d",
                      cache_package->idx, cache_package->sub_idx);
      return cache_package;
    } else {
     return NULL;
    }
  }
}

void PcapProcessor::flush_cache_parse_data_() {
  InnoDataPacket *innopkt;
  for (auto &item : map_dispather_info_) {
    current_ip_folder_ = item.second.ip_path;
    current_pcd_folder_ = item.second.pcd_path;
    current_msg_fd_ = item.second.message_fd;
    current_status_fd_ = item.second.state_fd;
    current_cache_ = item.second.cache_data;
    size_t cache_size = current_cache_->size();
    for (size_t i = 0; i < cache_size; i++) {
      innopkt = reinterpret_cast<InnoDataPacket *>
                (current_cache_->back().data);
      inno_log_verify(innopkt != NULL, "innopkt = null");
      process_data_cpoint_(*innopkt);
      current_cache_->pop_back();
    }
    delete current_cache_;
    current_cache_ = NULL;
    inno_log_verify(current_status_fd_ >= 0, "current_status_fd_ is 0");
    close(current_status_fd_);
    current_status_fd_ = -1;
    inno_log_verify(current_msg_fd_ >= 0, "current_msg_fd_ is 0");
    close(current_msg_fd_);
    current_msg_fd_ = -1;
  }
  map_dispather_info_.clear();
}

bool PcapProcessor::parse_message_(char *data, int len) {
  InnoDataPacket *pkt = reinterpret_cast<InnoDataPacket *>(data);
  message_counter++;
  inno_log_trace("parse_message message_counter = %lu\n", message_counter);
  // sanity check after relase-2.0.0-rc138
  if (!InnoDataPacketUtils::check_data_packet(*pkt, 0)) {
    inno_log_error("corrupted pkt->idx = %lu", pkt->idx);
    error_message_counter++;
    return false;
  }

  InnoMessage *msg = &pkt->messages[0];
  uint32_t level = msg->level;
  uint32_t code = msg->code;
  inno_log_trace("level = %u code = %u\n", level, code);
  switch (level) {
    case INNO_MESSAGE_LEVEL_INFO:
      inno_log_trace("Info content = %s\n", msg->content);
      break;
    case INNO_MESSAGE_LEVEL_WARNING:
      inno_log_trace("warning content = %s\n", msg->content);
      break;
    case INNO_MESSAGE_LEVEL_ERROR:
      inno_log_trace("error content = %s\n", msg->content);
      break;
    case INNO_MESSAGE_LEVEL_CRITICAL:
      inno_log_trace("critical content = %s\n", msg->content);
      break;
    case INNO_MESSAGE_LEVEL_FATAL:
      inno_log_trace("fatal content = %s\n", msg->content);
      break;
    default:
      inno_log_trace("debug content = %s\n", msg->content);
      break;
  }
  int write_len;
  write_len = snprintf(msg_buffer_, sizeof(msg_buffer_),
                    "%s\n", msg->content);
  int r = write(current_msg_fd_, msg_buffer_, write_len);
  if (r < 0) {
    inno_log_warning("cannot write data to current_msg_fd_ %d.",
                                              current_msg_fd_);
  }
  return true;
}


//
//
//
bool PcapProcessor::parse_raw_(char *data, int len) {
  if (len < Raw4UdpHeader::kHeaderSize) {
    return false;
  }

  Raw4UdpHeader header;
  if (!InnoDataPacketUtils::raw4_header_from_net(data, len, &header)) {
    return false;
  }

  //
  if (raw_fd_map_.find(header.idx) == raw_fd_map_.end()) {
    raw_fd_map_[header.idx] =
        new RawDataMessage(current_ip_folder_, header.idx);
  }

  // save or cache
  raw_fd_map_[header.idx]->save(header, data, len);

  if (raw_fd_map_[header.idx]->has_finished()) {
    raw_fd_map_[header.idx]->close();
    raw_fd_map_.erase(header.idx);
  }

  return true;
}

void PcapProcessor::counter_total_(bool ret) {
  if (ret) {
    total_successfully_counter++;
  } else {
    total_failed_counter++;
  }
}

void PcapProcessor::open_current_statue_msg_filename() {
  std::string current_msg_filename_;
  std::string current_status_filename_;
  current_status_filename_ = current_ip_folder_ + "/status_log.txt";
  current_status_fd_ = open(current_status_filename_.c_str(),
                    O_WRONLY | O_CREAT | O_APPEND, 0644);
  inno_log_verify(current_status_fd_ >= 0,
                  "cannot open %s", current_status_filename_.c_str());
  current_msg_filename_ = current_ip_folder_ + "/message_log.txt";
  current_msg_fd_ = open(current_msg_filename_.c_str(),
                O_WRONLY | O_CREAT | O_APPEND, 0644);
  inno_log_verify(current_msg_fd_ >= 0, "cannot open %s",
                  current_msg_filename_.c_str());
}


int PcapProcessor::create_folder_(const char* path) {
  if (path == NULL) {
    inno_log_error("the path is NULL!");
  }
  if (access(path, 0) != 0) {
    if (mkdir(path, 0755) == -1) {
      inno_log_error("mkdir folder  = %s error.", path);
      return -1;
    }
  } else {
    // do thing
  }
  return 0;
}

int PcapProcessor::set_current_ip_dispatcher(unsigned int saddr) {
  int ret = 0;
  struct in_addr addr;
  std::unordered_map<int, DispatcherInfo>::iterator iter;
  iter = map_dispather_info_.find(saddr);
  if (iter != map_dispather_info_.end()) {
    if (current_ip_folder_ != iter->second.ip_path) {
      current_ip_folder_ = iter->second.ip_path;
      current_pcd_folder_ = iter->second.pcd_path;
      current_msg_fd_ = iter->second.message_fd;
      current_status_fd_ = iter->second.state_fd;
      current_cache_ = iter->second.cache_data;
    }
  } else {
    DispatcherInfo dispather_info;
    std::list<CacheData> *new_cache = new std::list<CacheData>;
    if (new_cache == NULL) {
      return -1;
    }

    addr.s_addr = saddr;
    current_ip_folder_ = inet_ntoa(addr);
    current_ip_folder_ = output_foldername_ + "/" + current_ip_folder_;
    inno_log_trace("create current_ip_folder_ = %s",
                                       current_ip_folder_.c_str());
    ret = create_folder_(current_ip_folder_.c_str());
    if (ret < 0) {
      return -1;
    }
    open_current_statue_msg_filename();
    current_pcd_folder_ = current_ip_folder_ + "/pcd";
    ret = create_folder_(current_pcd_folder_.c_str());
    if (ret < 0) {
      return -1;
    }
    current_cache_ = new_cache;
    dispather_info.cache_data = current_cache_;
    dispather_info.ip_path = current_ip_folder_;
    dispather_info.pcd_path = current_pcd_folder_;
    dispather_info.message_fd = current_msg_fd_;
    dispather_info.state_fd = current_status_fd_;
    map_dispather_info_[saddr] = dispather_info;
  }
  return 0;
}

void PcapProcessor::parse_inno_package(uint16_t destport, char *data, int len) {
  bool result = false;
  total_receive_counter++;
  inno_log_trace("total_receive_counter = %lu\n", total_receive_counter);
  if (destport == RAW_PORT) {
    parse_raw_(data, len);
    return;
  } else {
    InnoStatusPacket *statusPkt = reinterpret_cast<InnoStatusPacket *>(data);
    if (statusPkt->common.version.magic_number ==
        kInnoMagicNumberStatusPacket) {
      result = parse_status_(data, len);
      counter_total_(result);
      return;
    } else if (statusPkt->common.version.magic_number ==
               kInnoMagicNumberDataPacket) {
      InnoDataPacket *pkt = reinterpret_cast<InnoDataPacket *>(data);
      if (pkt->type == INNO_ITEM_TYPE_MESSAGE ||
          pkt->type == INNO_ITEM_TYPE_MESSAGE_LOG) {
        result = parse_message_(data, len);
        counter_total_(result);
        return;
      } else if (pkt->type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD ||
                 pkt->type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
        result = parse_data_(data, len);
        counter_total_(result);
        return;
      }
    }
  }

  total_failed_counter++;
  inno_log_info("Magic numbe or type error.\n");
  return;
}

bool PcapProcessor::check_offset_(uint16_t expect_off, uint16_t offset) {
  if (expect_off != offset) {
      partial_sub_frame_counter++;
      inno_log_warning("Miss the UDP package expect offset =%d, "
                       "but the offset = %d.",
                       expect_off, offset);
      return false;
  }
  return true;
}

void PcapProcessor::process_udp(u_char *arg, const struct pcap_pkthdr *pcap_pkt,
                               const u_char *packet, uint32_t saddr) {
  static char pt_DFbuffer[kMaxIPPackageSize];
  static char pt_buffer[kMaxBlockSize];
  static bool udp_head = false;
  static uint64_t block_len = 0;
  static uint64_t index_udp = 0;
  static uint16_t expect_offset = 0;

  index_udp++;
  PcapProcessor *pcap = reinterpret_cast<PcapProcessor *>(arg);
  int result = pcap->set_current_ip_dispatcher(saddr);
  if (result < 0) {
    return;
  }

  inno_log_trace("No. = %ld\n", index_udp);
  struct iphdr *ipptr = (struct iphdr*)(packet + sizeof(struct ether_header));
  struct udphdr *udpheader = (struct udphdr *)
                (packet + sizeof(struct ether_header) + sizeof(struct iphdr));
  uint16_t frag_off = ntohs(ipptr->frag_off);
  uint16_t tot_len = ntohs(ipptr->tot_len);
  char *dataheader = reinterpret_cast<char*>(udpheader)
                      + sizeof(struct udphdr);
  uint16_t ihl = ipptr->ihl * 4;
  uint16_t offset = (frag_off & 0x1fff) << 3;
  uint16_t pt_DFbuffer_len = 0;

  uint16_t destport = ntohs(udpheader->dest);
  bool ret = false;
  if ((frag_off & 0x4000) &&
      (destport == PORT || destport == RAW_PORT)) {  // DF
    pt_DFbuffer_len = tot_len - ihl - sizeof(struct udphdr);
    memcpy(pt_DFbuffer, dataheader, pt_DFbuffer_len);
    pcap->parse_inno_package(destport, pt_DFbuffer, pt_DFbuffer_len);
  } else if (frag_off & 0x2000) {   // MF
    ret = pcap->check_offset_(expect_offset, offset);
    if (udp_head && ret) {
      memcpy(pt_buffer + block_len,
             reinterpret_cast<char*>(udpheader), tot_len - ihl);
      block_len += tot_len - ihl;
      expect_offset += tot_len - ihl;
    } else if (destport == PORT || destport == RAW_PORT) {
      udp_head = true;
      uint16_t data_len = tot_len - ihl - sizeof(struct udphdr);
      // prevent exception caused by imcomplete frame
      if (data_len > 0 && data_len <= sizeof(pt_buffer) - block_len) {
        memcpy(pt_buffer + block_len, dataheader, data_len);
        block_len += data_len;
        expect_offset += tot_len - ihl;
      } else {
        inno_log_warning(
            "Current udp package data_len invalid. Miss UDP package.");
        return;
      }
    } else {
      return;
    }
  } else if ((frag_off & 0xe000) == 0) {
    ret = pcap->check_offset_(expect_offset, offset);
    if (udp_head && ret) {   // last data package
      udp_head = false;
      memcpy(pt_buffer + block_len,
             reinterpret_cast<char*>(udpheader), tot_len - ihl);
      block_len += tot_len - ihl;
      pcap->parse_inno_package(destport, pt_buffer, block_len);

    } else if (destport == PORT) {
       inno_log_warning("Don't get the UDP last package. Miss UDP package.");
    } else {
      block_len = 0;
      expect_offset = 0;
      return;
    }
    block_len = 0;
    expect_offset = 0;
  }
}

void PcapProcessor::process_pcap(u_char *user,
                  const struct pcap_pkthdr *pkthdr,
                  const u_char *packet) {
  struct iphdr *ipptr;
  struct ether_header *eptr;
  u_char protocol;

  // get the ether header
  eptr = (struct ether_header*)packet;
  if (ntohs(eptr->ether_type) != ETHERTYPE_IP) {
    return;
  }
  // get the ip header
  ipptr = (struct iphdr*)(packet + sizeof(struct ether_header));
  protocol = ipptr->protocol;
  if (IPPROTO_UDP == protocol) {
    PcapProcessor::process_udp(user, pkthdr, packet, ipptr->saddr);
  }
}

bool handler_pcap_(std::string filename, u_char *user) {
  char err_buf[PCAP_ERRBUF_SIZE] = "";
  // struct pcap_pkthdr pkthdr = { 0 };
  pcap_t *pcap_ptr = pcap_open_offline(filename.c_str(), err_buf);
  if (!pcap_ptr) {
      inno_log_error("pcap_open_offline failed for file %s: error msg:%s",
                                  filename.c_str(), err_buf);
      return false;
  }

  pcap_dispatch(pcap_ptr, 0, PcapProcessor::process_pcap,
                reinterpret_cast<u_char*>(user));
  pcap_close(pcap_ptr);
  return true;
}



/***********************
 * class DatProcessor
 ***********************/
class DatProcessor:public PcapProcessor {
 public:
  DatProcessor(const std::string &dat_filename,
                    const std::string &output_foldername,
                    const ParseConfig &config)
      : PcapProcessor(dat_filename, output_foldername, config) {
      }
  ~DatProcessor() {}
  int process_dat(int fd);
};

int DatProcessor::process_dat(int fd) {
  LidarHeader lidar_header;
  TranscationHeader trans_header;
  char lidar_data[1024 * 10];

  int ret = read(fd, &lidar_header, sizeof(lidar_header));
  if (ret <= 0) {
     inno_log_error("can't read fd %d", fd);
     return -1;
  }

  if (strcmp(lidar_header.info.name, "LidarDt") != 0
     && strcmp(lidar_header.info.name, "LidarSt") != 0) {
    inno_log_error("lidar_header name = %s not LidarDt or LidarSt",
                                    lidar_header.info.name);
    return -2;
  }

  inno_log_trace("lidar_header receive address = %d.%d.%d.%d",
                            (lidar_header.dest.recv_addr >> 24) & 0xff,
                            (lidar_header.dest.recv_addr >> 16) & 0xff,
                            (lidar_header.dest.recv_addr >> 8) & 0xff,
                            (lidar_header.dest.recv_addr) & 0xff);
  inno_log_trace("lidar_header dest recv_addr= %d",
                                lidar_header.dest.rece_port);
  while (0 != (ret = read(fd, &trans_header, sizeof(trans_header)))) {
    inno_log_trace(" trans_header send_timestamp= %ld",
                                    trans_header.send_timestamp);
    inno_log_trace(" trans_header recv_timestamp = %ld",
                                    trans_header.recv_timestamp);
    inno_log_trace(" trans_header size = %ld",
                                    trans_header.size);
    ret = read(fd, lidar_data, trans_header.size);
    if (ret <= 0) {
      return 0;
    }
    total_receive_counter++;
    int result = set_current_ip_dispatcher(lidar_header.dest.recv_addr);
    if (result < 0) {
      return -3;
    }
    parse_inno_package(lidar_header.dest.rece_port,
                       lidar_data,
                       trans_header.size);
  }
  return 0;
}

bool handler_dat_(std::string filename, DatProcessor *user) {
  int ret = 0;
  int fd_dat = open(filename.c_str(), O_RDONLY);
  if (fd_dat < 0) {
    inno_log_error("open %s failed", filename.c_str());
    return false;
  } else {
    inno_log_info("start to parse %s", filename.c_str());
    ret = user->process_dat(fd_dat);
    if (ret < 0) {
       inno_log_error("process_dat failed ret = %d", ret);
    }
  }
  close(fd_dat);
  return true;
}

/***********************
 * usage()
 ***********************/
void usage(const char *arg0) {
  fprintf(stderr,
          "Usage:\n"
          "   %s "
          "[-i <input pcap file>] "
          "[-o <output folder>]"
          "[--debug <log_level>]"
          "[--record-pcd <0 no record, default 1 record>]\n", arg0);
  return;
}

/***********************
 * main()
 ***********************/
int main(int argc, char **argv) {
  if (argc < 5) {
    usage(argv[0]);
    exit(0);
  }
  std::string filename;
  std::string output_foldername = "falcon_lidar_out";
  ParseConfig config;
  enum InnoLogLevel debug_level = INNO_LOG_LEVEL_INFO;
  config.record_pcd = 1;

  /***********************
   * parse command line
   ***********************/

  /* getopt_long stores the option index here. */
  int c;
  struct option long_options[] = {
    /* These options set a flag. */
    {"input-file", required_argument, 0, 'i'},
    {"output-fold", required_argument, 0, 'o'},
    {"debug", required_argument, 0, 'd'},
    {"record-pcd", required_argument, 0, 'r'},
    {"help", no_argument, NULL, 'h'},
    {0, 0, 0, 0},
  };
  const char *optstring = "i:o:h:d:r";
  while (1) {
    int option_index = 0;
    c = getopt_long(argc, argv, optstring, long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1) {
      break;
    }

    switch (c) {
      case 0:
        /* If this option set a flag, do nothing else now. */
        if (long_options[option_index].flag != 0) {
          break;
        }
        inno_log_verify(optarg == NULL, "option %s with arg %s",
                        long_options[option_index].name, optarg);
        break;

      case 'h':
        usage(argv[0]);
        exit(0);
        break;

      case 'd':
        debug_level = (enum InnoLogLevel)atoi(optarg);
        break;

      case 'r':
        config.record_pcd = atoi(optarg);
        break;

      case 'i':
        filename = optarg;
        break;

      case 'o':
        {
          output_foldername = optarg;
          std::string command;
          command = "mkdir -p " + output_foldername;
          int ret = system(command.c_str());
          if (ret < 0) {
            inno_log_error("Can't create the folder.");
            usage(argv[0]);
            exit(1);
          }
        }
        break;

      case '?':
        abort();

      default:
        inno_log_error("unknown options %c\n", c);
        usage(argv[0]);
        exit(1);
    }
  }

  /***********************
   * set debug level
   ***********************/
  inno_lidar_set_log_level(debug_level);

  size_t pos = filename.find(".dat");
  if (pos != filename.npos) {
    DatProcessor processor(filename, output_foldername, config);
    handler_dat_(filename, &processor);
  } else {
    /***********************
     * create PcapProcessor object
     ***********************/
    PcapProcessor processor(filename, output_foldername, config);
    handler_pcap_(filename, reinterpret_cast<u_char*>(&processor));
  }

  return 0;
}
