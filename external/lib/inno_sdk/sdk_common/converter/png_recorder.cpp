/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk_common/converter/png_recorder.h"

#include "sdk_common/converter/png.h"
#include "sdk_common/converter/png_range_color.h"
#include "sdk_common/converter/png_range_image.h"
#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_packet_utils.h"
#include "utils/inno_lidar_log.h"

namespace innovusion {

constexpr int INVALID_FRAME_IDX = -1;

//
//
//
PngRecorder::PngRecorder(size_t job_duration, size_t max_pack_num,
                         size_t max_block_num, bool ignore_first_frame) {
  inno_log_info("++++++++ PngRecorder ++++++++");

  job_duration_ = job_duration;

  max_pack_num_ = max_pack_num;
  max_block_num_ = max_block_num;

  block_so_far_ = 0;
  packet_so_far_ = 0;

  //
  frame_idx_last_ = INVALID_FRAME_IDX;

  frame_captured_ = 0;
  current_point_in_frame_ = 0;

  //
  size_t len_saved_ =
      sizeof(InnoDataPacket) + sizeof(InnoBlock2) * max_block_num;
  saved_packet_ =
      reinterpret_cast<InnoDataPacket *>(calloc(len_saved_, 1));
  inno_log_verify(saved_packet_,
                  "cannot allocate saved_ %" PRI_SIZEU "",
                  len_saved_);
  memset(saved_packet_, 0, len_saved_);

  size_t header_len_saved = sizeof(InnoDataPacket) * max_pack_num;
  saved_packet_headers_ =
      reinterpret_cast<InnoDataPacket *>(calloc(header_len_saved, 1));
  inno_log_verify(saved_packet_headers_,
                  "cannot allocate saved_header %" PRI_SIZEU "",
                  header_len_saved);
  memset(saved_packet_headers_, 0, header_len_saved);
  if (!ignore_first_frame) {
    start_capture_();
  }
}

//
//
//
PngRecorder::~PngRecorder() {
  inno_log_info("-------- ~PngRecorder --------");

  if (saved_packet_) {
    free(saved_packet_);
  }

  if (saved_packet_headers_) {
    free(saved_packet_headers_);
  }
}

void PngRecorder::start_capture_() {
  frame_captured_ = 0;
  current_point_in_frame_ = 0;

  // switch
  this->state_ = State::Capturing;
}

//
//
//
void PngRecorder::switch_state_(const InnoDataPacket* pkt) {
  bool is_first_subframe =
      frame_idx_last_ >= 0 &&
      frame_idx_last_ != ssize_t(pkt->idx);

  switch (this->state_) {
    case State::Invalid:
      if (is_first_subframe) {
        inno_log_info("capture first frame %" PRI_SIZEU "",
                      pkt->idx);
        start_capture_();
      }
      break;

    case State::Capturing:
      if (is_first_subframe) {
        inno_log_info("capture %u points in frame-%" PRI_SIZEU
                      "-%" PRI_SIZED " to png",
                      current_point_in_frame_, frame_captured_,
                      frame_idx_last_);

        frame_captured_++;
        current_point_in_frame_ = 0;

        if (job_duration_ > 0 &&
            frame_captured_ >= job_duration_) {
          this->state_ = State::Saving;
        }
      }
      break;

    default:
      break;
  }

  //
  frame_idx_last_ = pkt->idx;
}

//
//
//
void PngRecorder::capture_packet_(const InnoDataPacket *pkt) {
  inno_log_trace("capture frame %" PRI_SIZEU "",
                 pkt->idx);
  if (packet_so_far_ >= max_pack_num_) {
    inno_log_warning("reach limit kMaxPacketNum %u", packet_so_far_);
    return;
  }

  uint32_t block_left = max_block_num_ - block_so_far_;
  uint32_t block_to_copy = pkt->item_number;
  if (block_to_copy > block_left) {
    block_to_copy = block_left;
    inno_log_warning("reach limit kMaxBlockNum %" PRI_SIZEU "",
                     max_block_num_);
  }

  current_point_in_frame_ += InnoDataPacketUtils::get_points_count(*pkt);
  for (uint32_t i = 0; i < block_to_copy; i++) {
    if (pkt->multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
      memcpy(saved_packet_->inno_block2s + block_so_far_, &pkt->inno_block1s[i],
             sizeof(InnoBlock1));
    } else {
      memcpy(saved_packet_->inno_block2s + block_so_far_, &pkt->inno_block2s[i],
             sizeof(InnoBlock2));
    }
    block_so_far_++;
  }

  if (block_to_copy > 0) {
    memcpy(saved_packet_headers_ + packet_so_far_, pkt, sizeof(InnoDataPacket));
    packet_so_far_++;
  }
  return;
}

//
//
//
bool PngRecorder::capture(const InnoDataPacket *pkt) {
  inno_log_verify(pkt, "pkt");
  switch_state_(pkt);

  if (this->state_ == State::Capturing) {
    capture_packet_(pkt);
  }

  return this->state_ == State::Saving;
}

//
//
//
void PngRecorder::save_point_to_image_(RangeImage* range_image) {
  inno_log_verify(range_image != NULL, "range_image");

  InnoBlock2 *block;
  size_t current_packet = 0;

  for (size_t i = 0; i < block_so_far_; i++) {
    inno_log_verify(current_packet < packet_so_far_,
                    "%" PRI_SIZEU " vs %u",
                    current_packet, packet_so_far_);

    block = &saved_packet_->inno_block2s[i];

    InnoBlockFullAngles full_angles;
    InnoDataPacketUtils::get_block_full_angles(&full_angles, block->header);

    for (uint32_t ch = 0; ch < kInnoChannelNumber; ch++) {
      for (uint32_t m = 0; m < 2; m++) {
        InnoChannelPoint &pt = block->points[InnoBlock2::get_idx(ch, m)];
        if (pt.radius > 0) {
          InnoXyzrD xyzr;
          InnoDataPacketUtils::get_xyzr_meter(full_angles.angles[ch], pt.radius,
                                              ch, &xyzr);

          range_image->insert_point(xyzr.x, xyzr.y, xyzr.z, pt.refl);
        }
      }
    }
  }
}

//
//
//
void PngRecorder::save_png_(SaveFunc save_func) {
  inno_log_info("save_to_png, block_so_far_ : %u", block_so_far_);

  //
  float angularResolution_x = static_cast<float>(0.05f * (M_PI / 180.0f));
  float angularResolution_y = static_cast<float>(0.05f * (M_PI / 180.0f));

  float maxAngleWidth = static_cast<float>(120.0f * (M_PI / 180.0f));
  float maxAngleHeight = static_cast<float>(60.0f * (M_PI / 180.0f));

  RangeImage range_image(angularResolution_x, angularResolution_y,
                         maxAngleWidth, maxAngleHeight);

  range_image.start();
  save_point_to_image_(&range_image);
  range_image.stop();

  // save
  RangeColor colorScale(ColorType::BGYR);
  PNGWriter pngWriter(range_image.width, range_image.height,
                      PNGColorType::RGB, 8);

  range_image.get_image(
      [&colorScale](float range, float ref, unsigned char* r,
                    unsigned char* g, unsigned char* b) {
        colorScale.getColor(range, ref, r, g, b);
      },
      [&pngWriter](int x, int y, unsigned char r, unsigned char g,
                    unsigned char b) { pngWriter.encode(x, y, r, g, b); });

  save_func(pngWriter);

  // switch state
  this->state_ = State::Finish;
}

//
//
//
void PngRecorder::save(std::vector<char>* buf) {
  inno_log_verify(buf, "png buf");

  save_png_([buf](const PNGWriter& pngWriter){
    inno_log_info("saveto_buffer");
    pngWriter.saveto_buffer(buf);
  });
}

//
//
//
void PngRecorder::save(const std::string& filename) {
  inno_log_verify(filename.empty() == false, "png filename is empty.");

  std::string filename_ = filename;

  if (!filename_.empty()) {
    if (filename_.rfind(".") == std::string::npos) {
      filename_ += ".png";
    }
  }

  save_png_([&filename_](const PNGWriter& pngWriter){
    inno_log_info("saveto_file");
    pngWriter.saveto_file(filename_);
  });
}

}  // namespace innovusion
