/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <sys/time.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>

#include <limits>
#include <string>

#include "src/sdk_common/converter/cframe_converter.h"
#include "src/sdk_common/converter/png_recorder.h"
#include "src/sdk_common/converter/rosbag_recorder.h"
#include "src/sdk_common/inno_lidar_api.h"
#include "src/sdk_common/inno_lidar_packet_utils.h"
#include "src/utils/inno_lidar_log.h"
#include "src/utils/utils.h"

static const double kUsInSecond = 1000000.0;
static const double k10UsInSecond = 100000.0;
static const uint32_t kMaxMsgBuf = 1500;

/***********************
 * class FileRecorder
 ***********************/
class FileRecorder {
 public:
  enum FileType {
    FILE_TYPE_PCD,
    FILE_TYPE_PCD_BINARY,
    FILE_TYPE_CSV,
    FILE_TYPE_INNO_PC,
    FILE_TYPE_INNO_PC_XYZ,
    FILE_TYPE_INNO_CFRAME,
    FILE_TYPE_BAG,
    FILE_TYPE_PNG,
    FILE_TYPE_MAX,
  };

  DEFINE_INNO_COMPACT_STRUCT(PcdPoint) {
    float x;
    float y;
    float z;
    uint16_t intensity;
    uint8_t channel;
    uint8_t roi;
    uint8_t facet;
    uint8_t is_2nd_return;
    uint8_t multi_return;
    uint8_t confid_level;
    uint8_t flags;
    uint8_t elongation;
    double timestamp;
    uint16_t scanline;
    uint16_t scan_idx;
    uint32_t frame_id;
    uint32_t ring_id;
  };

 public:
  static const size_t kMaxBufferSize = 256 * 1024;
  static const size_t kHeaderSize = 400;
  static const size_t kMaxCsvSize_ = 150;
  static const size_t kMaxBlockNum = 1000 * 1000;
  static const size_t kMaxPacketNum = 1000 * 100;

 public:
  explicit FileRecorder(const std::string &filename,
                        const uint64_t frame_id,
                        bool reflectance,
                        enum FileType file_type)
      : filename_(filename)
      , frame_id_(frame_id)
      , fd_(-1)
      , point_count_(0)
      , reflectance_(reflectance)
      , file_type_(file_type)
      , buffer_cursor_(0)
      , byte_written_(0)
      , cframe_converter_(NULL)
      , rosbag_stream_(NULL)
      , png_stream_(NULL) {
    if (can_record_cframe()) {
      cframe_converter_ = new innovusion::CframeConverter();
      inno_log_verify(cframe_converter_, "cframe_converter_");
    } else if (can_record_bag()) {
      rosbag_stream_ = new innovusion::RosbagRecorder(
          filename.c_str(),
          NULL, NULL, -1);
      inno_log_verify(rosbag_stream_, "rosbag_stream_");
    } else if (can_record_png()) {
      png_stream_ = new innovusion::PngRecorder(
          0, kMaxPacketNum, kMaxBlockNum, false);
      inno_log_verify(png_stream_, "png_stream_");
    }

    if (can_record_bag()) {
      fd_ = -1;
    } else {
      fd_ = innovusion::InnoUtils::open_file(filename_.c_str(),
                                             O_WRONLY | O_CREAT | O_TRUNC,
                                             0644);
      inno_log_verify(fd_ >= 0, "cannot open %s", filename_.c_str());
      inno_log_info("open file %s to record frame %" PRI_SIZEU "",
                    filename_.c_str(), frame_id);
      write_dummy_header_();
    }
    return;
  }

  ~FileRecorder() {
    if (can_record_cframe() && cframe_converter_) {
      inno_cframe_header *cframe = cframe_converter_->close_current_frame();
      if (cframe) {
        write_buffer_(cframe, cframe->get_size());
      }
    }
    if (rosbag_stream_) {
      rosbag_stream_->add_block(NULL);
      delete rosbag_stream_;
      rosbag_stream_ = NULL;
    }
    if (png_stream_) {
      std::vector<char> buf;
      buf.reserve(1024 * 1024 * 6);
      png_stream_->save(&buf);
      int ret = write(fd_, buf.data(), buf.size());
      if (ret < ssize_t(buf.size())) {
        inno_log_error_errno("write to file failed %d", ret);
      }
      delete png_stream_;
      png_stream_ = NULL;
    }
    close_();
    if (fd_ >= 0) {
      close(fd_);
    }
    fd_ = -1;
    if (cframe_converter_) {
      delete cframe_converter_;
      cframe_converter_ = NULL;
    }
    return;
  }

  bool can_record_packet() const {
    return file_type_ == FILE_TYPE_INNO_PC ||
        file_type_ == FILE_TYPE_INNO_PC_XYZ;
  }

  bool can_record_cframe() const {
    return file_type_ == FILE_TYPE_INNO_CFRAME;
  }

  bool can_record_bag() const {
    return file_type_ == FILE_TYPE_BAG;
  }

  bool can_record_png() const {
    return file_type_ == FILE_TYPE_PNG;
  }

  InnoDataPacket *convert_to_data_xyz_point_(const InnoDataPacket &pkt) {
    // 1. calculate max size and allocate new data packet
    size_t new_pkt_size = InnoDataPacketUtils::get_data_packet_size(
        INNO_ITEM_TYPE_XYZ_POINTCLOUD,
        InnoDataPacketUtils::get_max_points_count(pkt),
        INNO_MULTIPLE_RETURN_MODE_NONE);

    InnoDataPacket *new_pkt =
        reinterpret_cast<InnoDataPacket *>(malloc(new_pkt_size));
    inno_log_verify(new_pkt, "new_pkt");

    // 2. convert INNO_ITEM_TYPE_SPHERE_POINTCLOUD to
    //    INNO_ITEM_TYPE_XYZ_POINTCLOUD
    bool ret = InnoDataPacketUtils::convert_to_xyz_pointcloud(pkt,
                                                              new_pkt,
                                                              new_pkt_size,
                                                              false);
    inno_log_verify(ret, "ret");
    return new_pkt;
  }

  void add_data_packet(const InnoDataPacket &pkt_in) {
    if (fd_ < 0) {
      return;
    }
    const InnoDataPacket *packet = NULL;
    InnoDataPacket *new_packet = NULL;
    if (file_type_ == FILE_TYPE_INNO_PC) {
      inno_log_verify(pkt_in.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD,
                      "cannot convert type=%d to inno_pc",
                      pkt_in.type);
      packet = &pkt_in;
    } else if (file_type_ == FILE_TYPE_INNO_PC_XYZ) {
      if (pkt_in.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
        new_packet =
            InnoDataPacketUtils::convert_to_xyz_pointcloud_malloced(pkt_in);
        inno_log_verify(new_packet, "cannot convert");
        packet = new_packet;
      } else if (pkt_in.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
        packet = &pkt_in;
      } else {
        return;
      }
    } else {
      inno_log_panic("invalid file type %d", file_type_);
    }

    inno_log_verify(InnoDataPacketUtils::check_data_packet(*packet,
                                                           packet->common.size),
                    "corrupted pkt");
    write_buffer_(packet, packet->common.size);

    if (new_packet) {
      free(new_packet);
    }
    return;
  }

  void add_data_packet_to_cframe(const InnoDataPacket &pkt) {
    if (fd_ < 0) {
      return;
    }
    inno_cframe_header *cframe =
        cframe_converter_->add_data_packet(&pkt, 1);
    if (cframe) {
      write_buffer_(cframe, cframe->get_size());
    }
    return;
  }

  void add_data_packet_to_bag(const InnoDataPacket &pkt) {
    rosbag_stream_->add_block(&pkt);
    return;
  }

  void add_data_packet_to_png(const InnoDataPacket &pkt) {
    png_stream_->capture(&pkt);
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
                  const uint32_t scan_idx,
                  const uint32_t is_2nd_return,
                  const uint32_t ring_id) {
    if (fd_ < 0) {
      return;
    }

    int wri;
    const char *row_format;
    switch (file_type_) {
      case FILE_TYPE_CSV:
      case FILE_TYPE_PCD:
        if (buffer_cursor_ + 200 > kMaxBufferSize) {
          flush_buffer_();
        }
        row_format = file_type_ == FILE_TYPE_CSV
                   ? "%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
                     "%.5f,%u,%u,%" PRI_SIZEU ",%u\n "
                   : "%.3f %.3f %.3f %u %u %u %u %u %u %u %u %u "
                     "%.5f %u %u %" PRI_SIZEU " %u\n";
        wri = snprintf(buffer_ + buffer_cursor_,
                       sizeof(buffer_) - buffer_cursor_,
                       row_format,
                       x, y, z, ref, channel, in_roi,
                       facet, is_2nd_return, m_ret, confidence_level,
                       flags, elongation,
                       timestamp_sec, scanline, scan_idx, frame_id, ring_id);
        break;

      case FILE_TYPE_PCD_BINARY: {
        wri = sizeof(PcdPoint);
        if (buffer_cursor_ + wri > kMaxBufferSize) {
          flush_buffer_();
        }
        PcdPoint *pt = reinterpret_cast<PcdPoint *>(buffer_ + buffer_cursor_);
        pt->x = x;
        pt->y = y;
        pt->z = z;
        pt->intensity = ref;
        pt->channel = channel;
        pt->roi = in_roi;
        pt->facet = facet;
        pt->is_2nd_return = is_2nd_return;
        pt->multi_return = m_ret;
        pt->confid_level = confidence_level;
        pt->flags = flags;
        pt->elongation = elongation;
        pt->timestamp = timestamp_sec;
        pt->scanline = scanline;
        pt->scan_idx = scan_idx;
        pt->frame_id = frame_id;
        pt->ring_id = ring_id;
        break;
      }
      default:
        inno_log_panic("invalid type %d", file_type_);
    }

    buffer_cursor_ += wri;
    inno_log_verify(buffer_cursor_ <= sizeof(buffer_),
                    "not enough buffer");
    point_count_++;
    return;
  }

 private:
  void write_dummy_header_() {
    size_t write_size = 0;
    switch (file_type_) {
      case FILE_TYPE_CSV:
        write_size = kMaxCsvSize_;
        break;
      case FILE_TYPE_PCD:
      case FILE_TYPE_PCD_BINARY:
        write_size = sizeof(header_buffer_);
        break;
      case FILE_TYPE_INNO_PC:
      case FILE_TYPE_INNO_PC_XYZ:
      case FILE_TYPE_INNO_CFRAME:
      case FILE_TYPE_BAG:
      case FILE_TYPE_PNG:
        return;
      default:
        inno_log_panic("invalid type %d", file_type_);
    }
    memset(header_buffer_, ' ', write_size);
    header_buffer_[write_size - 1] = '\n';
    int r = write(fd_, header_buffer_, write_size);
    inno_log_verify(r >= 0, "cannot write header %s", filename_.c_str());
  }

  void rewrite_header_() {
    const char *pcd_header =
        "# .PCD v.7 - Point Cloud Data file format\n"
        "FIELDS "
        "x y z %s channel roi facet is_2nd_return multi_return confid_level "
        "flag elongation timestamp "
        "scanline scan_idx frame_id ring_id\n"
        "SIZE 4 4 4 2 1 1 1 1 1 1 1 1 8 2 2 4 4\n"
        "TYPE F F F U U U U U U U U U F U U U U\n"
        "COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n"
        "WIDTH %" PRI_SIZEU "\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %" PRI_SIZEU "\n"
        "DATA %s";
    const char *csv_header =
        "x,y,z,%s,channel,roi,facet,is_2nd_return,multi_return,confid_level,"
        "flag,elongation,timestamp,"
        "scanline,scan_idx,frame_id,ring_id";
    int r;
    uint32_t max_write_size = 0;

    switch (file_type_) {
      case FILE_TYPE_CSV:
        r = snprintf(header_buffer_, sizeof(header_buffer_),
                   csv_header,
                   reflectance_ ? "reflectance" : "intensity");
        max_write_size = kMaxCsvSize_ - 1;
        break;
      case FILE_TYPE_PCD:
      case FILE_TYPE_PCD_BINARY:
        r = snprintf(header_buffer_, sizeof(header_buffer_),
                     pcd_header, reflectance_ ? "reflectance" : "intensity",
                     point_count_, point_count_,
                     file_type_ == FILE_TYPE_PCD ? "ascii" : "binary");
        max_write_size = sizeof(header_buffer_) - 1;
        break;
      case FILE_TYPE_INNO_PC:
      case FILE_TYPE_INNO_PC_XYZ:
      case FILE_TYPE_INNO_CFRAME:
      case FILE_TYPE_BAG:
      case FILE_TYPE_PNG:
        return;
      default:
        inno_log_panic("invalid type %d", file_type_);
    }

    inno_log_verify(r < int32_t(sizeof(header_buffer_)),
                    "buffer too small %d vs %" PRI_SIZEU "",
                    r, sizeof(header_buffer_));
    inno_log_info("write %" PRI_SIZEU " points to pcd file %s",
                  point_count_,
                  filename_.c_str());
    FILE *file = fopen(filename_.c_str(), "r+");
    inno_log_verify(file != NULL, "Unable to open file %s", filename_.c_str());

    fseek(file, 0, SEEK_SET);
    size_t write_size = strlen(header_buffer_);
    inno_log_verify(write_size <= max_write_size,
                    "write_size %" PRI_SIZEU " > header_size %u",
                    write_size, max_write_size);
    size_t fr = fwrite(header_buffer_, 1, write_size, file);
    inno_log_verify(fr == strlen(header_buffer_),
                    "fwrite return %" PRI_SIZEU "", fr);
    fclose(file);
    return;
  }

  void write_buffer_(const void *src, size_t src_len) {
    if (buffer_cursor_ + src_len > kMaxBufferSize) {
      flush_buffer_();
      if (src_len >= kMaxBufferSize) {
        int r = write(fd_, src, src_len);
        inno_log_verify(r >= 0, "cannot write data to %s",
                        filename_.c_str());
        return;
      }
    }

    memcpy(buffer_ + buffer_cursor_, src, src_len);
    buffer_cursor_ += src_len;
    inno_log_verify(buffer_cursor_ < sizeof(buffer_),
                    "not enough buffer");
    byte_written_ += src_len;
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
  enum FileType file_type_;
  uint64_t buffer_cursor_;
  uint64_t byte_written_;
  char buffer_[kMaxBufferSize];
  char header_buffer_[kHeaderSize];
  innovusion::CframeConverter *cframe_converter_;
  innovusion::RosbagRecorder *rosbag_stream_;
  innovusion::PngRecorder *png_stream_;
};

/***********************
 * class ExampleProcessor
 ***********************/
class ExampleProcessor {
 public:
  ExampleProcessor(const std::string &filename,
                   const int64_t frame_start,
                   const int64_t frame_number,
                   const int64_t file_number,
                   int use_xyz_point,
                   std::string latency_file,
                   int ascii_pcd,
                   int extract_message)
      : filename_(filename)
      , frame_start_(frame_start)
      , frame_number_(frame_number)
      , file_number_(file_number)
      , use_xyz_point_(use_xyz_point)
      , latency_file_(latency_file)
      , current_frame_(-1)
      , frame_so_far_(-1)
      , file_so_far_(0)
      , file_recorder_(NULL)
      , file_type_(FileRecorder::FILE_TYPE_PCD)
      , done_(false) {
    // create status and message file
    size_t dot = filename_.find_last_of(".");
    if (dot != std::string::npos) {
      filename_base_ = filename_.substr(0, dot);
      file_extension_ = filename_.substr(dot);
      if (strcasecmp(file_extension_.c_str(), ".csv") == 0) {
        file_type_ = FileRecorder::FILE_TYPE_CSV;
      } else if (strcasecmp(file_extension_.c_str(),
                            ".inno_pc") == 0) {
        file_type_ = FileRecorder::FILE_TYPE_INNO_PC;
      } else if (strcasecmp(file_extension_.c_str(),
                            ".inno_pc_xyz") == 0) {
        file_type_ = FileRecorder::FILE_TYPE_INNO_PC_XYZ;
      } else if (strcasecmp(file_extension_.c_str(),
                            ".inno_cframe") == 0) {
        file_type_ = FileRecorder::FILE_TYPE_INNO_CFRAME;
      } else if (strcasecmp(file_extension_.c_str(),
                            ".bag") == 0) {
        file_type_ = FileRecorder::FILE_TYPE_BAG;
      } else if (strcasecmp(file_extension_.c_str(),
                            ".png") == 0) {
        file_type_ = FileRecorder::FILE_TYPE_PNG;
        inno_log_info("force to use sphere coordinate to record PNG file");
        use_xyz_point_ = 0;
      } else {
        file_type_ = ascii_pcd ?
                     FileRecorder::FILE_TYPE_PCD :
                     FileRecorder::FILE_TYPE_PCD_BINARY;
      }
    } else {
      filename_base_ = filename_;
      file_type_ = ascii_pcd ?
                   FileRecorder::FILE_TYPE_PCD :
                   FileRecorder::FILE_TYPE_PCD_BINARY;
    }
    if (extract_message) {
      status_filename_ = filename_ + "_status_log.txt";
      status_fd_ = innovusion::InnoUtils::open_file(
          status_filename_.c_str(),
          O_WRONLY | O_CREAT | O_APPEND, 0644);
      inno_log_verify(status_fd_ >= 0,
                      "cannot open %s", status_filename_.c_str());
      msg_filename_ = filename_ + "_message_log.txt";
      msg_fd_ = innovusion::InnoUtils::open_file(
          msg_filename_.c_str(),
          O_WRONLY | O_CREAT | O_APPEND, 0644);
      inno_log_verify(msg_fd_ >= 0, "cannot open %s", msg_filename_.c_str());
    } else {
      status_fd_ = -1;
      msg_fd_ = -1;
    }
    galvo_check_fd_ = -1;
    max_distance_fd_ = -1;
  }

  ~ExampleProcessor() {
    if (file_recorder_) {
      delete file_recorder_;
      file_recorder_ = NULL;
    }
    if (msg_fd_ >= 0) {
      close(msg_fd_);
      msg_fd_ = -1;
    }
    if (status_fd_ >= 0) {
      close(status_fd_);
      status_fd_ = -1;
    }
    if (galvo_check_fd_ >= 0) {
      close(galvo_check_fd_);
      galvo_check_fd_ = -1;
    }
    if (max_distance_fd_ >= 0) {
      close(max_distance_fd_);
      max_distance_fd_ = -1;
    }
    inno_log_info("----------Summary----------\n"
      "frame_counter = %" PRI_SIZEU ", "
      "miss_frame_counter = %" PRI_SIZEU ", "
      "miss_sub_frame_gap_count = %" PRI_SIZEU ", "
      "empty_sub_frame_count = %" PRI_SIZEU ", "
      "miss_sub_frame_last_one_counter = %" PRI_SIZEU ", "
      "miss_sub_frame_except_last_one_counter = %" PRI_SIZEU "\n",
      summary_package_.get_frame_count(),
      summary_package_.get_miss_frame_count(),
      summary_package_.get_miss_sub_frame_gap_count(),
      summary_package_.get_empty_sub_frame_count(),
      summary_package_.get_miss_sub_frame_last_one_count(),
      summary_package_.get_empty_sub_frame_except_last_one_count());
  }

  void set_done() {
    done_ = true;
  }

  bool is_done() {
    return done_;
  }

  void save_message_to_file(const char* message) {
    if (msg_fd_ >= 0) {
      int write_len = snprintf(msg_buffer_, sizeof(msg_buffer_),
                           "%s\n", message);
      int r = write(msg_fd_, msg_buffer_, write_len);
      if (r < 0) {
        inno_log_warning("cannot write data to %s", msg_filename_.c_str());
      }
    }
  }

  void save_galvo_check_result_to_file(const char* message) {
    if (galvo_check_fd_ >= 0) {
      int write_len = snprintf(galvo_check_buffer_,
                           sizeof(galvo_check_buffer_),
                           "%s\n", message);
      int r = write(galvo_check_fd_, galvo_check_buffer_, write_len);
      if (r < 0) {
        inno_log_warning("cannot write data to %s",
                             galvo_check_filename_.c_str());
      }
    }
  }

  void save_max_distance_result_to_file(const char* message) {
    if (max_distance_fd_ >= 0) {
      int write_len = snprintf(max_distance_buffer_,
                           sizeof(max_distance_buffer_),
                           "%s\n", message);
      int r = write(max_distance_fd_, max_distance_buffer_, write_len);
      if (r < 0) {
        inno_log_warning("cannot write data to %s",
                             max_distance_filename_.c_str());
      }
    }
  }

  void create_galvo_check_result_file() {
    time_t rawtime;
    struct tm *tm_info;
    char temp[80];
    struct tm result_tm;
    time(&rawtime);
#ifndef __MINGW64__
    tm_info = localtime_r(&rawtime, &result_tm);
#else
    tm_info = localtime_s(&result_tm, &rawtime) == 0 ?
             &result_tm : NULL;
#endif
    strftime(temp, sizeof(temp) - 1, "%Y%m%d-%H%M%S", tm_info);
    galvo_check_filename_ = "galvo-check-";
    galvo_check_filename_.append(temp).append(".txt");
    galvo_check_fd_ = innovusion::InnoUtils::open_file(
          galvo_check_filename_.c_str(),
          O_WRONLY | O_CREAT | O_APPEND, 0644);
    inno_log_verify(galvo_check_fd_ >= 0, "cannot open %s",
                              galvo_check_filename_.c_str());
    const char *galvo_check_head_ = "Frame\tC_Code\tP_Count\t" \
                                    "Ref_x\tRef_y\tRef_z\t" \
                                    "CoeA\tCoeB\tCoeC\tCoeD\t" \
                                    "Rot_x\tRot_y\tRot_z\t" \
                                    "Speed\tSpeed_Acc\tR2\tCos(θ)\t" \
                                    "D_Angle\tMean_Angle\tAngle_R2\t" \
                                    "Fault_Status\tF_Times\t" \
                                    "Valid_Count\tSpell_Time(us)";
    save_galvo_check_result_to_file(galvo_check_head_);
  }

  void create_max_distance_result_file() {
    time_t rawtime;
    struct tm *tm_info;
    char temp[80];
    struct tm result_tm;
    time(&rawtime);
#ifndef __MINGW64__
    tm_info = localtime_r(&rawtime, &result_tm);
#else
    tm_info = localtime_s(&result_tm, &rawtime) == 0 ?
             &result_tm : NULL;
#endif
    strftime(temp, sizeof(temp) - 1, "%Y%m%d-%H%M%S", tm_info);
    max_distance_filename_ = "max-distance-";
    max_distance_filename_.append(temp).append(".txt");
    max_distance_fd_ = innovusion::InnoUtils::open_file(
          max_distance_filename_.c_str(),
          O_WRONLY | O_CREAT | O_APPEND, 0644);
    inno_log_verify(max_distance_fd_ >= 0, "cannot open %s",
                              max_distance_filename_.c_str());
  }

  static void message_callback_s(const int lidar_handle, void *ctx,
                                 const uint32_t from_remote,
                                 const enum InnoMessageLevel level,
                                 const enum InnoMessageCode code,
                                 const char *error_message) {
    inno_log_info("level = %d code = %d\n", level, code);
    switch (level) {
      case INNO_MESSAGE_LEVEL_INFO:
        inno_log_info("content = %s\n", error_message);
        if (code == INNO_MESSAGE_CODE_READ_FILE_END) {
          reinterpret_cast<ExampleProcessor *>(ctx)->set_done();
        }
        break;
      case INNO_MESSAGE_LEVEL_WARNING:
        inno_log_warning("content = %s\n", error_message);
        break;
      case INNO_MESSAGE_LEVEL_ERROR:
        inno_log_error("content = %s\n", error_message);
        break;
      case INNO_MESSAGE_LEVEL_FATAL:
        inno_log_fatal("content = %s\n", error_message);
        break;
      case INNO_MESSAGE_LEVEL_CRITICAL:
        if (code == INNO_MESSAGE_CODE_CANNOT_READ) {
          inno_log_error("critical error, abort.");
          reinterpret_cast<ExampleProcessor *>(ctx)->set_done();
          break;
        } else {
          inno_log_fatal("content = %s\n", error_message);
          break;
        }
      default:
        inno_log_info("content = %s\n", error_message);
        break;
    }
    if (code == INNO_MESSAGE_CODE_GALVO_MIRROR_CHECK_RESULT) {
      reinterpret_cast<ExampleProcessor *>(ctx)->
                             save_galvo_check_result_to_file(error_message);
    } else if (code == INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT) {
      reinterpret_cast<ExampleProcessor *>(ctx)->
                             save_max_distance_result_to_file(error_message);
    } else {
      reinterpret_cast<ExampleProcessor *>(ctx)->
                                        save_message_to_file(error_message);
    }
    return;
  }

  void save_status_to_file(const InnoStatusPacket *pkt) {
    if (status_fd_ >= 0) {
      int write_len = InnoDataPacketUtils::printf_status_packet(
          *pkt, status_buffer_, sizeof(status_buffer_));

      if (write_len > 0) {
        int r = write(status_fd_, status_buffer_, write_len);
        if (r < 0) {
          inno_log_warning("cannot write data to %s", status_filename_.c_str());
        }
      }
    }
  }

  static int status_callback_s(const int lidar_handle, void *ctx,
                               const InnoStatusPacket *pkt) {
    inno_log_verify(pkt, "pkt");

    // sanity check
    if (!InnoDataPacketUtils::check_status_packet(*pkt, 0)) {
      inno_log_error("corrupted pkt->idx = %" PRI_SIZEU "", pkt->idx);
      return 0;
    }

    static size_t cnt = 0;
    if (cnt++ % 100 == 1) {
      constexpr size_t buf_size = 2048;
      char buf[buf_size]{0};

      int ret = InnoDataPacketUtils::printf_status_packet(*pkt, buf, buf_size);
      if (ret > 0) {
        inno_log_info("Received status packet #%" PRI_SIZEU ": %s", cnt, buf);
      } else {
        inno_log_warning("Received status packet #"
                         "%" PRI_SIZEU ": errorno: %d", cnt, ret);
      }
    }

    reinterpret_cast<ExampleProcessor *>(ctx)->save_status_to_file(pkt);
    return 0;
  }


  static void get_latency_info(const InnoDataPacket &pkt,
                               const std::string latency_file) {
    int64_t latency = 0;
    struct timeval tv{};
    gettimeofday(&tv, nullptr);
    int64_t cur_time = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
    latency = cur_time - pkt.common.ts_start_us;
    std::fstream f;
    f.open(latency_file, std::ios::out|std::ios::app);
    f << "frame_idx=" << pkt.idx << "," << "latency=" <<
      latency << "," << "ts_start_us=" << (int64_t) pkt.common.ts_start_us
      << "," << "accept_time=" << cur_time << "," <<
      "item_count=" << pkt.item_number << std::endl;
    f.close();
  }

  static int data_callback_s(const int lidar_handle, void *ctx,
                             const InnoDataPacket *pkt) {
    static size_t cnt = 0;
    if (cnt++ % 1000 == 0) {
      inno_log_info("Received data packet #%" PRI_SIZEU "\n", cnt);
    }
    inno_log_verify(pkt, "pkt");

    // sanity check
    if (pkt->type != INNO_ITEM_TYPE_SPHERE_POINTCLOUD &&
        pkt->type != INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
      inno_log_warning("ignore type %u", pkt->type);
      return 0;
    }

    // sanity check
    if (!InnoDataPacketUtils::check_data_packet(*pkt, 0)) {
      inno_log_warning("corrupted pkt");
      return 0;
    }

    reinterpret_cast<ExampleProcessor *>(ctx)->process_data_(*pkt);

    return 0;
  }

  void set_ring_id_converter(RingIdConverterInterface *conveter) {
    ring_id_conveter_ = conveter;
  }

 private:
  inline FileRecorder *get_recorder_(const int64_t frame_id,
                                     const bool reflectance) {
    // how many frames we have seen so far?
    if (frame_so_far_ == -1 || current_frame_ != frame_id) {
      current_frame_ = frame_id;
      frame_so_far_++;
    }

    if (filename_.size() == 0) {
      // do nothing
    } else if (frame_number_ == -1) {
      // do nothing
    } else if (frame_so_far_ < frame_start_) {
      // do nothing
    } else if (frame_so_far_ < frame_start_ +
               frame_number_ * file_number_) {
      // in record range
      if (frame_so_far_ >= frame_start_ +
          frame_number_ * file_so_far_) {
        if (file_recorder_) {
          delete file_recorder_;
          file_recorder_ = NULL;
        }
      }
      if (file_recorder_ == NULL) {
        // create FileRecorder
        std::string fn = filename_;
        if (file_number_ != 1) {
          fn = filename_base_ + "-" +
               std::to_string(file_so_far_) +
               file_extension_;
        }
        file_recorder_ = new FileRecorder(fn,
                                          frame_id,
                                          reflectance,
                                          file_type_);
        file_so_far_++;
        inno_log_verify(file_recorder_ != NULL,
                        "cannot create file recorder");
      }
    } else {
      // record done, close the recorder
      if (file_recorder_) {
        delete file_recorder_;
        file_recorder_ = NULL;
      }
      set_done();
    }
    return file_recorder_;
  }

  void process_data_(const InnoDataPacket &pkt) {
    if (latency_file_ != "" && get_recorder_(pkt.idx, pkt.use_reflectance)) {
        get_latency_info(pkt, latency_file_);
    }

    // we have two ways of processing the data packet
    if (pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
      if (use_xyz_point_ == 0 ||
          (use_xyz_point_ == 1 &&
           file_type_ == FileRecorder::FILE_TYPE_INNO_PC)) {
        process_data_cpoint_(pkt);
      } else if (use_xyz_point_ == 1) {
        convert_and_process_data_xyz_point_(pkt);
      } else {
        inno_log_verify(false,
                        "invalid use_xyz_point number %d",
                        use_xyz_point_);
      }
    } else if (pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
      process_data_xyz_point_(pkt);
    } else {
      inno_log_panic("invalid type %d", pkt.type);
    }

    if (frame_so_far_ < frame_start_) {
      // not in record range yet
    } else if ((filename_.size() == 0) ||
               (frame_number_ == -1) ||
               (frame_so_far_ <
               frame_start_ + frame_number_ * file_number_)) {
      // summary date package frame and sub frame
      summary_package_.summary_data_package(pkt);
    }
  }

  void process_data_xyz_point_(const InnoDataPacket &pkt) {
    inno_log_verify(pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD,
                    "invalid packet %u", pkt.type);
    double frame_timestamp_sec = pkt.common.ts_start_us / kUsInSecond;
    // enumerate each xyz_point in the new packet and add it to the recorder
    FileRecorder *recorder = get_recorder_(pkt.idx, pkt.use_reflectance);
    if (recorder) {
      if (recorder->can_record_packet()) {
        recorder->add_data_packet(pkt);
      } else if (recorder->can_record_cframe()) {
        recorder->add_data_packet_to_cframe(pkt);
      } else if (recorder->can_record_bag()) {
        recorder->add_data_packet_to_bag(pkt);
      } else if (recorder->can_record_png()) {
        recorder->add_data_packet_to_png(pkt);
      } else {
        for (uint32_t i = 0; i < pkt.item_number; i++) {
          const InnoXyzPoint &pt = pkt.xyz_points[i];
          recorder->add_points(pkt.idx, pt.x, pt.y, pt.z,
                               pt.refl, pt.channel,
                               pt.in_roi,
                               pt.facet,
                               pt.is_2nd_return,
                               pkt.confidence_level,
                               pt.type, pt.elongation,
                               frame_timestamp_sec +
                               pt.ts_10us / k10UsInSecond,
                               pt.scan_id,
                               pt.scan_idx,
                               pt.is_2nd_return,
                               pt.ring_id);
        }
      }
    }
    return;
  }

  /*
    this function shows how to convert SPHERE_POINTCLOUD packet to
    XYZ_POINTCLOUD packet, and then enumerate each xyz_point in the new
    packet, then add it to the file recorder
  */
  void convert_and_process_data_xyz_point_(const InnoDataPacket &pkt) {
    InnoDataPacket *new_pkt = InnoDataPacketUtils::
            convert_to_xyz_pointcloud_malloced(pkt, ring_id_conveter_);
    inno_log_verify(new_pkt, "new_pkt");

    // process
    process_data_xyz_point_(*new_pkt);

    // 4. free new packet
    free(new_pkt);
  }

  /*
    this function shows how to enumerate blocks and points in the packet
    directly, and use InnoDataPacketUtils::get_xyzr_meter to get each
    point's x,y,z coodindate, then add it to the file recorder
  */
  void process_data_cpoint_(const InnoDataPacket &pkt) {
    uint32_t return_number;
    uint32_t unit_size;
    FileRecorder *recorder = get_recorder_(pkt.idx, pkt.use_reflectance);

    if (!recorder) {
      return;
    }

    if (recorder->can_record_bag()) {
      recorder->add_data_packet_to_bag(pkt);
      return;
    } else if (recorder->can_record_png()) {
      recorder->add_data_packet_to_png(pkt);
      return;
    } else if (recorder->can_record_packet()) {
      recorder->add_data_packet(pkt);
      return;
    }

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
            recorder->add_points(pkt.idx, xyzr.x, xyzr.y, xyzr.z,
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
                                 pt.is_2nd_return,
                                 ring_id_conveter_ ? ring_id_conveter_->
                                 get_ring_id(static_cast<InnoLidarMode>(
                                             pkt.common.lidar_mode),
                                             pkt.scanner_direction,
                                             block->header.scan_id,
                                             channel) : 0);
          }
        }
      }
    }
    return;
  }

 private:
  std::string filename_;
  std::string filename_base_;
  int64_t frame_start_;
  int64_t frame_number_;
  int64_t file_number_;
  int use_xyz_point_;
  std::string latency_file_;
  int generate_latency_info_;
  int64_t current_frame_;
  int64_t frame_so_far_;
  int file_so_far_;
  FileRecorder *file_recorder_;
  std::string msg_filename_;
  std::string status_filename_;
  std::string galvo_check_filename_;
  std::string max_distance_filename_;
  int status_fd_;
  int msg_fd_;
  int galvo_check_fd_;
  int max_distance_fd_;
  char msg_buffer_[kMaxMsgBuf];
  char status_buffer_[kMaxMsgBuf];
  char galvo_check_buffer_[kMaxMsgBuf];
  char max_distance_buffer_[kMaxMsgBuf];
  enum FileRecorder::FileType file_type_;
  std::string file_extension_;
  bool done_;
  InnoSummaryPackage summary_package_;
  RingIdConverterInterface *ring_id_conveter_{nullptr};
};

/***********************
 * usage()
 ***********************/
void usage(const char *arg0) {
  inno_fprintf(2,
          "Usage:\n"
          "   %s\n"
          "\t{[--lidar-ip <LIDAR_IP>]\n"
          "\t\t[--lidar-port <LIDAR_PORT>]\n"
          "\t\t[--lidar-udp-port <LIDAR_UDP_PORT>]\n"
          "\t\t[--use-tcp]\n"
          "\t\t[--lidar-mode <LIDAR_MODE 3=normal/5=calibration>]\n"
          "\t\t[--reflectance <REFLECTANCE_MODE "
               "1=intensity/2=reflectance>]\n"
          "\t\t[--multireturn <MULTI_RETURN_MODE "
               "1=single/2=two-strongest/3=strongest+furthest>]\n"
          "\t\t[--falcon-eye <HORIZONTAL_DEGREE -60 to 60>,"
               "<VERTICAL_DEGREE -20 to 20>] |\n"
          "\t  [--inno-pc-filename <INPUT_INNO_PC_FILENAME>]}\n"
          "\t[--use-xyz-point <0,1,2>]\n"
          "\t[--file-number <NUMBER_OF_FILE>]\n"
          "\t[--frame-start <Nth_FRAME_TO_RECORD>]\n"
          "\t[--frame-number <NUMBER_OF_FRAME_TO_RECORD>]\n"
          "\t[--output-filename <OUTPUT_FILENAME."
               "pcd|csv|inno_pc|inno_pc_xyz|inno_cframe|bag|png>]\n"
          "\t[--ascii-pcd]\n"
          "\t[--extract-message]\n", arg0);
  inno_fprintf(2,
          "\n"
          "Examples:\n"
          " --record 2 frames from live LIDAR to test.pcd (LIDAR "
          "is configured to multicast or broadcast).\n"
          "   %s --frame-number 2 --output-filename test.pcd\n\n"
          " --record 100 frames from live LIDAR via UDP "
          "to test.inno_pc_xyz (LIDAR "
          "is NOT configured to multicast or broadcast).\n"
          "   %s --lidar-udp-port 8010 "
          "--frame-number 100 --output-filename test.inno_pc_xyz\n\n"
          " --record 100 frames from live LIDAR via TCP "
          "to test.inno_pc (LIDAR "
          "is NOT configured to multicast or broadcast).\n"
          "   %s --lidar-udp-port 0 "
          "--frame-number 100 --output-filename test.inno_pc\n\n"
          " --record 1 frames per pcd, start from frame #10, "
          "for 20 frames "
          "from input.inno_pc file to test-xx.pcd\n"
          "   %s --inno-pc-filename input.inno_pc "
          "--frame-start 10 --frame-number 1 "
          "--file-number 20 "
          "--output-filename test.pcd\n\n"
          "Please see more usage examples in test_get_pcd.bash\n",
          arg0, arg0, arg0, arg0);
  return;
}

/***********************
 * main()
 ***********************/
int main(int argc, char **argv) {
  if (argc == 1) {
    usage(argv[0]);
    exit(0);
  }

  /***********************
   * set debug level
   ***********************/
  enum InnoLogLevel debug_level = INNO_LOG_LEVEL_INFO;
  inno_lidar_set_log_level(debug_level);

  /***********************
   * parse command line
   ***********************/
  std::string inno_pc_filename;
  std::string lidar_ip = "172.168.1.10";
  uint16_t lidar_port = 8010;
  uint16_t lidar_udp_port = 0;
  std::string filename;
  int64_t frame_start = -1;
  int64_t frame_number = 1;
  int64_t file_number = 1;
  int use_xyz_point = 1;
  std::string latency_file = "";
  int extract_message = 0;
  int use_tcp = 0;
  int ascii_pcd = 0;
  int vehicle_speed = 0;
  int max_distance = 0;
  enum InnoLidarMode lidar_mode = INNO_LIDAR_MODE_NONE;
  enum InnoReflectanceMode reflectance = INNO_REFLECTANCE_MODE_NONE;
  enum InnoMultipleReturnMode multireturn = INNO_MULTIPLE_RETURN_MODE_NONE;
  double roi_center_h = 0;
  double roi_center_v = 0;
  bool roi_set = false;
  int simulation_galvo_check = 0;

  /* getopt_long stores the option index here. */
  int c;
  struct option long_options[] = {
    /* These options set a flag. */
    {"lidar-ip", required_argument, 0, 'n'},
    {"lidar-port", required_argument, 0, 'p'},
    {"lidar-udp-port", required_argument, 0, 'O'},
    {"frame-start", required_argument, 0, 's'},
    {"frame-number", required_argument, 0, 'c'},
    {"file-number", required_argument, 0, 'N'},
    {"output-filename", required_argument, 0, 'P'},
    {"inno-pc-filename", required_argument, 0, 'f'},
    {"falcon-eye", required_argument, 0, 'e'},
    {"lidar-mode", required_argument, 0, 'm'},
    {"reflectance", required_argument, 0, 'F'},
    {"multireturn", required_argument, 0, 'M'},
    {"use-xyz-point", required_argument, 0, 'x'},
    {"galvo-check", required_argument, 0, 'g'},
    {"vehicle-speed", required_argument, 0, 'v'},
    {"extract-message", no_argument, &extract_message, 1},
    {"use-tcp", no_argument, &use_tcp, 1},
    {"ascii-pcd", no_argument, &ascii_pcd, 1},
    {"max-distance", no_argument, &max_distance, 1},
    {"latency-file", required_argument, 0, 'l'},
    {"help", no_argument, NULL, 'h'},
    {0, 0, 0, 0}
  };
  const char *optstring = "ac:e:f:F:hn:l:m:M:N:O:p:P:s:x:g:v:";
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

      case 'a':
        ascii_pcd = 1;
        break;

      case 'h':
        usage(argv[0]);
        exit(0);
        break;

      case 'P':
        filename = optarg;
        break;

      case 'f':
        inno_pc_filename = optarg;
        break;

      case 'n':
        lidar_ip = optarg;
        break;

      case 's':
        frame_start = atoll(optarg);
        break;

      case 'c':
        frame_number = atoll(optarg);
        if (frame_number <= 0) {
          frame_number = std::numeric_limits<int>::max();
        }
        break;

      case 'F':
        reflectance = (enum InnoReflectanceMode)atoi(optarg);
        if (reflectance != INNO_REFLECTANCE_MODE_INTENSITY &&
            reflectance != INNO_REFLECTANCE_MODE_REFLECTIVITY) {
          usage(argv[0]);
          exit(1);
        }
        break;

      case 'M':
        multireturn = (enum InnoMultipleReturnMode)atoi(optarg);
        if (multireturn != INNO_MULTIPLE_RETURN_MODE_SINGLE &&
            multireturn != INNO_MULTIPLE_RETURN_MODE_2_STRONGEST &&
            multireturn != INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
          usage(argv[0]);
          exit(1);
        }
        break;

      case 'm':
        lidar_mode = (enum InnoLidarMode)atoi(optarg);
        if (lidar_mode != INNO_LIDAR_MODE_WORK_NORMAL &&
            lidar_mode != INNO_LIDAR_MODE_WORK_CALIBRATION &&
            lidar_mode != INNO_LIDAR_MODE_WORK_QUIET) {
          usage(argv[0]);
          exit(1);
        }
       break;

      case 'N':
        file_number = atoll(optarg);
        if (file_number <= 0) {
          file_number = std::numeric_limits<int>::max();
        }
       break;

      case 'e':
        if (sscanf(optarg, "%lf,%lf",
                   &roi_center_h,
                   &roi_center_v) == 2) {
          roi_set = true;
        } else {
          inno_log_error("invalid --falcon-eye option %s",
                         optarg);
          exit(1);
        }
        break;

      case 'p':
        lidar_port = atoi(optarg);
        break;

      case 'O':
        lidar_udp_port = strtoul(optarg, NULL, 0);
        break;

      case 'x':
        use_xyz_point = atoi(optarg);
        break;

      case 'l':
        latency_file = optarg;
        break;

      case 'g':
        simulation_galvo_check = atoi(optarg);
        break;

      case 'v':
        vehicle_speed = atoi(optarg);
        break;
      case '?':
        abort();

      default:
        inno_log_error("unknown options %c\n", c);
        usage(argv[0]);
        exit(1);
    }
  }

  if (frame_start == -1) {
    frame_start = inno_pc_filename.size() > 0 ? 0 : 1;
  }

  /***********************
   * create ExampleProcessor object
   ***********************/
  ExampleProcessor processor(filename, frame_start,
                             frame_number, file_number,
                             use_xyz_point, latency_file, ascii_pcd,
                             extract_message);

  /***********************
   * open lidar handle
   ***********************/
  int handle;
  if (inno_pc_filename.size() > 0) {
    handle = inno_lidar_open_file(
        "file", /* name of lidar instance */
        inno_pc_filename.c_str(),
        false, /* inno_pc format                        */
        0,     /* as fast as possible without drop data */
        0,     /* play once, no rewind                  */
        0);    /* no skip                               */
  } else {
    handle = inno_lidar_open_live("live", /* name of lidar instance */
                                  lidar_ip.c_str(),
                                  lidar_port,
                                  use_tcp ?
                                  INNO_LIDAR_PROTOCOL_PCS_TCP :
                                  INNO_LIDAR_PROTOCOL_PCS_UDP,
                                  lidar_udp_port);
  }
  inno_log_verify(handle > 0, "cannot open lidar");

  // the code shows 3 ways to process the data callback
  if (use_xyz_point == 0) {
    // callback with INNO_ITEM_TYPE_SPHERE_POINTCLOUD or
    // INNO_ITEM_TYPE_XYZ_POINTCLOUD and
    // enumerate each block and channel points
  } else if (use_xyz_point == 1) {
    // callback with INNO_ITEM_TYPE_SPHERE_POINTCLOUD and
    // covert to INNO_ITEM_TYPE_XYZ_POINTCLOUD
    // then enumerate each xyz points
  } else if (use_xyz_point == 2) {
    // set SDK to callback with INNO_ITEM_TYPE_XYZ_POINTCLOUD and
    // then enumerate each xyz points
    inno_lidar_set_attribute_string(handle,
                                    "force_xyz_pointcloud", "1");
  } else {
    inno_log_verify(false,
                    "invalid use_xyz_point number, "
                    "must in [0, 1, 2]");
  }

  /***********************
   * set lidar reflectance/multi-return/roi/work mode
   ***********************/
  int ret;
  if (reflectance != INNO_REFLECTANCE_MODE_NONE) {
    ret = inno_lidar_set_reflectance_mode(handle, reflectance);
    inno_log_verify(ret == 0, "set_reflectance failed %d", ret);
  }

  if (multireturn != INNO_MULTIPLE_RETURN_MODE_NONE) {
    ret = inno_lidar_set_return_mode(handle, multireturn);
    inno_log_verify(ret == 0, "set_return_mode failed %d", ret);
  }

  if (roi_set) {
    ret = inno_lidar_set_roi(handle, roi_center_h, roi_center_v);
    inno_log_verify(ret == 0, "set_roi failed %d", ret);
  }

  inno_log_info("simulation_galvo_check=%d, max_distance=%d, vehicle_speed=%d",
                          simulation_galvo_check, max_distance, vehicle_speed);
  if (simulation_galvo_check) {
    ret = inno_lidar_set_attribute_string(handle,
                                       "vehicle_speed", "85.0");
    inno_log_verify(ret == 0, "set vehicle_speed failed %d", ret);
    ret = inno_lidar_set_attribute_string(handle,
                       "galvo_ext_ref", "-2.241,0.0094,-2.201,0,0,0");
    inno_log_verify(ret == 0, "set galvo_ext_ref failed %d", ret);
    processor.create_galvo_check_result_file();
  }

  if (max_distance) {
    processor.create_max_distance_result_file();
  }

  if (lidar_mode != INNO_LIDAR_MODE_NONE) {
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    ret = inno_lidar_set_mode(handle, lidar_mode,
                              &pre_mode, &status);
    inno_log_verify(ret == 0, "set_mode failed %d", ret);
  }

  /***********************
   * set lidar pointcloud processing callbacks
   ***********************/
  ret = inno_lidar_set_callbacks(handle,
                                 processor.message_callback_s,
                                 processor.data_callback_s,
                                 processor.status_callback_s,
                                 NULL,  // use default get_host_time()
                                 &processor);
  inno_log_verify(ret == 0, "set_callbacks failed %d", ret);

  /***********************
   * start lidar pointcloud processing threads
   ***********************/
  ret = inno_lidar_start(handle);
  inno_log_verify(ret == 0, "start failed %d", ret);

  ret = inno_lidar_set_attribute_string(handle, "use_ring_id", "1");
  if (ret == 0) {
    processor.set_ring_id_converter(inno_lidar_get_ring_id_converter(handle));
  } else {
    inno_log_warning("get ring id converter failed."
                     " Please ignore this if you needn't ring id.");
  }
  /***********************
   * wait until processor is done
   ***********************/
  while (!processor.is_done()) {
    usleep(10000);
    if (max_distance) {
      inno_lidar_set_attribute_string(handle,
                  "vehicle_speed", std::to_string(vehicle_speed).c_str());
    }
  }

  /***********************
   * stop lidar pointcloud processing threads
   ***********************/
  ret = inno_lidar_stop(handle);
  inno_log_verify(ret == 0, "stop failed %d", ret);

  /***********************
   * close lidar handle
   ***********************/
  ret = inno_lidar_close(handle);
  inno_log_verify(ret == 0, "close failed %d", ret);

  return 0;
}
