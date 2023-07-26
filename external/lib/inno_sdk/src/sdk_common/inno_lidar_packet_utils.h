/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef SDK_COMMON_INNO_LIDAR_PACKET_UTILS_H_
#define SDK_COMMON_INNO_LIDAR_PACKET_UTILS_H_

#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <string>

#include "sdk_common/inno_lidar_packet.h"
#include "utils/inno_lidar_log.h"
#include "sdk_common/ring_id_converter_interface.h"

// FUNC is in type InnoDataPacketPointsIterCallback
#define ITERARATE_INNO_DATA_PACKET_CPOINTS(FUNC, ctx, packet, count)       \
  do {                                                                     \
    uint32_t unit_size;                                                    \
    uint32_t mr;                                                           \
    mr = InnoDataPacketUtils::get_return_times(                            \
                  InnoMultipleReturnMode((packet)->multi_return_mode)); \
    if (mr == 2) {                                                      \
      unit_size = sizeof(InnoBlock2);                                   \
    } else if (mr == 1) {                                               \
      unit_size = sizeof(InnoBlock1);                                   \
    } else {                                                            \
      inno_log_panic("return times of return mode %d is %d?",           \
                      (packet)->multi_return_mode, mr);                 \
    }                                                                   \
    const InnoBlock *block = reinterpret_cast<const InnoBlock *>        \
                       (&(packet)->inno_block1s[0]);                    \
    for (size_t i = 0;                                                  \
         i < (packet)->item_number;                                     \
         i++, block = reinterpret_cast<const InnoBlock *>               \
                    (reinterpret_cast<const char *>(block) + unit_size)) { \
      InnoBlockFullAngles full_angles;                                  \
      InnoDataPacketUtils::get_block_full_angles(&full_angles,          \
                                                 block->header);        \
      for (uint32_t ch = 0; ch < kInnoChannelNumber; ch++) {            \
        for (uint32_t m = 0; m < mr; m++) {                             \
          const InnoChannelPoint &pt =                                  \
                             block->points[InnoBlock2::get_idx(ch, m)]; \
          FUNC(ctx, (*packet), (*block), pt, full_angles, ch, m);       \
          count++;                                                      \
        }                                                               \
      }                                                                 \
    }                                                                   \
  } while (0)

// FUNC is in type InnoDataPacketXYZPointsIterCallback
#define ITERARATE_INNO_DATA_PACKET_XYZ_POINTS(FUNC, ctx, packet)        \
  do {                                                                  \
    const InnoXyzPoint *inno_xyz_point =                                \
                                 reinterpret_cast<const InnoXyzPoint *> \
                                            (&(packet)->xyz_points[0]); \
    for (size_t i = 0;                                                  \
         i < (packet)->item_number;                                     \
         i++, inno_xyz_point++) {                                       \
      FUNC(ctx, (*packet), *inno_xyz_point);                            \
    }                                                                   \
  } while (0)

extern "C" {
  // 2 + 4 + 2 + 2 + 2 + 2 + 2 + 2 + 1 + 1 = 20
  DEFINE_INNO_COMPACT_STRUCT(InnoPcNpy) {
    uint16_t frame_id;   // need to + base_frame_id
    uint32_t ts_10us;    // need to + base_ts_10us
    int16_t h_angle_unit;
    int16_t v_angle_unit;
    uint16_t radius_unit_lo16;      // low 16 bits of radius in unit
    uint16_t radius_unit_hi1_refl;  // additional_high_bit_of_distance[15:15]
                                    // refl[0:14]
    uint16_t scanline_id;
    uint16_t idx_within_scanline;
    uint8_t flags_1;  // direction[7:7], facet[4:6], return[3:3],
                      // roi[2:2], channel[0:1]
    uint8_t flags_2;  // elongation[4:7], type[2:3], confidence_level[0:1]
  };

  DEFINE_INNO_COMPACT_STRUCT(InnoCaliData) {
    uint16_t frame_id;
    uint8_t  channel;
    uint8_t  facet;
    uint8_t  direction;
    uint8_t  roi;
    int16_t poly_angle;
    int16_t galvo_angle;
    int16_t  h_angle;
    int16_t  v_angle;
    uint16_t ref_intensity;
    uint32_t radius;
    uint32_t intensity;
    float    reflectance;
  };

  DEFINE_INNO_COMPACT_STRUCT(InnoCaliDataBuffer) {
    uint32_t item_number;
    InnoCaliData cali_data[0];
  };

  class InnoBlockAngles {
   public:
    int16_t h_angle;
    int16_t v_angle;
  };

  class InnoBlockFullAngles {
   public:
    InnoBlockAngles angles[kInnoChannelNumber];
  };

  class InnoDataPacketUtils {
    typedef void (*InnoDataPacketPointsIterCallback)(
        void *ctx,
        const InnoDataPacket &pkt,
        const InnoBlock &block,
        const InnoChannelPoint &pt,
        const InnoBlockFullAngles &angle,
        const uint16_t ch,
        const uint16_t m);

    typedef void (*InnoDataPacketXyzPointsIterCallback)(
        void *ctx,
        const InnoDataPacket &pkt,
        const InnoXyzPoint &pt);

   private:
    static int init_;
    static int v_angle_offset_[kInnoChannelNumber];
    static const uint32_t kVTableSizeBits_ = 4;
    static const uint32_t kHTableSizeBits_ = 6;
    static const uint32_t kVTableSize_ = 1 << kVTableSizeBits_;
    static const uint32_t kHTableSize_ = 1 << kHTableSizeBits_;
    static const uint32_t kVTableEffeHalfSize_ = 6;
    static const uint32_t kHTableEffeHalfSize_ = 22;
    static const uint32_t kXZSize_ = 2;
    static int8_t nps_adjustment_[kVTableSize_][kHTableSize_][kInnoChannelNumber][kXZSize_];  // NOLINT
    static const double kAdjustmentUnitInMeter_;

   private:
    static void lookup_xz_adjustment_(const InnoBlockAngles &angles,
                                      uint32_t ch, double *x, double *z);

   public:
    static int init_f(void);

    static inline void get_block_full_angles(InnoBlockFullAngles *full,
                                             const InnoBlockHeader &b) {
      full->angles[0].h_angle = b.h_angle;
      full->angles[0].v_angle = b.v_angle;
      full->angles[1].h_angle = b.h_angle + b.h_angle_diff_1;
      full->angles[1].v_angle = b.v_angle + b.v_angle_diff_1 +
                                v_angle_offset_[1];
      full->angles[2].h_angle = b.h_angle + b.h_angle_diff_2;
      full->angles[2].v_angle = b.v_angle + b.v_angle_diff_2 +
                                v_angle_offset_[2];
      full->angles[3].h_angle = b.h_angle + b.h_angle_diff_3;
      full->angles[3].v_angle = b.v_angle + b.v_angle_diff_3 +
                                v_angle_offset_[3];
    }

    /*
     * @brief Get the block size in bytes and number of returns in the pkt
     * @param pkt DataPacket
     * @param block_size_in_byte Return size (in bytes) of block in the pkt
     * @param number_return Return number of returns in the pkt
     * @return Void
     */
    static inline void get_block_size_and_number_return(
        const InnoDataPacket &pkt,
        uint32_t *block_size_in_byte,
        uint32_t *number_return) {
      inno_log_verify(pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD,
                      "invalid pkt type %u", pkt.type);
      if (pkt.multi_return_mode
          == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
          pkt.multi_return_mode
          == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        *block_size_in_byte = sizeof(InnoBlock2);
        *number_return = 2;
      } else if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
        *block_size_in_byte = sizeof(InnoBlock1);
        *number_return = 1;
      } else {
        inno_log_verify(false, "invalid return mode %u", pkt.multi_return_mode);
      }
      return;
    }

    /* @brief Calculate x/y/z coordinate based on angle and radius.
      *radius_meter = radius_unit * kMeterPerInnoDistanceUnit;
      double va = angles.v_angle * kRadPerInnoAngleUnit;
      double ha = angles.h_angle * kRadPerInnoAngleUnit;
      double t = *radius_meter * cos(va);
      *x_meter = *radius_meter * sin(va);
      *y_meter = t * sin(ha);
      *z_meter = t * cos(ha);
      Fine tune x_meter and z_meter in the end
     * @param angles Angles
     * @param radius_unit Radius in InnoDistanceUnit
     * @param channel Channel
     * @param result Store result.
     * @return Void
     */
    static void get_xyzr_meter(const InnoBlockAngles angles,
                               const uint32_t radius_unit,
                               const uint32_t channel,
                               InnoXyzrD *result);

    /*
     * @brief convert an InnoChannelPoint in a block to
              an InnoXyzPoint
     * @param block Block header
     * @param cp Source channel point
     * @param angles Angles
     * @param channel Channel
     * @param pt Destination InnoXyzPoint

     * @return number of points in the data packet. -1 if invalid item type
     */
    static inline void get_xyz_point(const InnoBlockHeader &block,
                                     const InnoChannelPoint &cp,
                                     const InnoBlockAngles angles,
                                     const uint32_t channel,
                                     InnoXyzPoint *pt) {
      InnoXyzrD xyzr;
      get_xyzr_meter(angles, cp.radius, channel, &xyzr);
      pt->x = xyzr.x;
      pt->y = xyzr.y;
      pt->z = xyzr.z;
      pt->radius = xyzr.radius;
      pt->ts_10us = block.ts_10us;
      pt->scan_idx = block.scan_idx;
      pt->scan_id = block.scan_id;
      pt->in_roi = block.in_roi;
      pt->facet = block.facet;
      pt->reserved_flags = block.reserved_flags;
      pt->refl = cp.refl;
      pt->type = cp.type;
      pt->elongation = cp.elongation;
      pt->channel = channel;
      pt->is_2nd_return = cp.is_2nd_return;
    }

    /*
     * @brief Enumerate each point in the data packet and make callback,
              only apply to INNO_ITEM_TYPE_SPHERE_POINTCLOUD
     * @param pkt Data packet
     * @param callback Callback
     * @param ctx Callback context
     * @return number of points in the data packet.
     */
    static inline ssize_t iterate_cpoints(
        const InnoDataPacket &pkt,
        InnoDataPacketPointsIterCallback callback,
        void *ctx) {
      inno_log_verify(pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD,
                      "invalid pkt type %u", pkt.type);
      size_t pcount = 0;
      ITERARATE_INNO_DATA_PACKET_CPOINTS(callback, ctx, &pkt, pcount);
      return pcount;
    }

    /*
     * @brief Enumerate each point in the data packet and make callback,
              only apply to INNO_ITEM_TYPE_XYZ_POINTCLOUD
     * @param pkt Data packet
     * @param callback Callback
     * @param ctx Callback context
     * @return number of points in the data packet.
     */
    static inline ssize_t iterate_xyz_points(
        const InnoDataPacket &pkt,
        InnoDataPacketXyzPointsIterCallback callback,
        void *ctx) {
      inno_log_verify(pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD,
                      "invalid pkt type %u", pkt.type);
      ITERARATE_INNO_DATA_PACKET_XYZ_POINTS(callback, ctx, &pkt);
      return pkt.item_number;
    }

    /*
     * @brief Return the data packet size according to the item type, count
              and return mode.
     * @param type Data packet type
     * @param item_count Item count
     * @param mode Multi-return mode
     * @return size of data packet in bytes. Return 0 if the type is invalid.
     */
    static inline size_t get_data_packet_size(InnoItemType type,
                                              uint32_t item_count,
                                              InnoMultipleReturnMode mode) {
      size_t unit_size;
      if (type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
        if (mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
          unit_size = sizeof(InnoBlock1);
        } else if (mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
                   mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
          unit_size = sizeof(InnoBlock2);
        } else {
          return 0;
        }
      } else if (type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
        unit_size = sizeof(InnoXyzPoint);
      } else {
        inno_log_verify(false, "bad type = %d", type);
      }
      return sizeof(InnoDataPacket) + item_count * unit_size;
    }

    /**
     * get true return times of a return mode, we assert the input mode is valid
     * If new return mode added, we may need to refine this function
     * @param mode
     * @return return times
     */
    static inline int get_return_times(const InnoMultipleReturnMode mode) {
      return mode >= INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ? 2 : 1;
    }

    /*
     * @brief Convert INNO_ITEM_TYPE_SPHERE_POINTCLOUD points the source
              data packet to INNO_ITEM_TYPE_XYZ_POINTCLOUD
     * @param src Source data packet, type must be
              INNO_ITEM_TYPE_SPHERE_POINTCLOUD
     * @return converted InnoDataPacket allocated by malloc.
               return NULL if malloc failed or invalid packet
     */
    static InnoDataPacket *convert_to_xyz_pointcloud_malloced(
        const InnoDataPacket &src,
        RingIdConverterInterface *ring_id_converter = NULL);

    /*
     * @brief Convert INNO_ITEM_TYPE_SPHERE_POINTCLOUD points the source
              data packet to INNO_ITEM_TYPE_XYZ_POINTCLOUD and add them
              to the destination data packet.
     * @param src Source data packet
     * @param dest Destination data packet
     * @param dest_size Max size of the destination data packet
     * @param append True if the new points are appended, false if start
     *        write from beginning
     * @return false if the pkt is invalid, true otherwise
     */
    static bool convert_to_xyz_pointcloud(const InnoDataPacket &src,
                                          InnoDataPacket *dest,
                                          size_t dest_size,
                                          bool append,
                                          RingIdConverterInterface
                                            *ring_id_converter = NULL);

    /*
     * @brief Sanity check the integrity of a InnoDataPacketGet.
     * @param pkt DataPacket
     * @param size Size of pkt if it is received from network or
     *        read from file
     * @return false if the pkt is invalid, true otherwise
     */
    static bool check_data_packet(const InnoDataPacket &pkt,
                                  size_t size);

    /*
     * @brief Sanity check the integrity of a InnoDataPacketGet.
     * @param pkt StatusPacket
     * @param size Size of pkt if it is received from network or
     *        read from file, 0 means don't check size
     * @return false if the pkt is invalid, true otherwise
     */
    static bool check_status_packet(const InnoStatusPacket &pkt,
                                    size_t size);

    /*
     * @brief InnoStatusPacket formatted output.
     * @param pkt StatusPacket
     * @param buffer
     * @param buffer_size
     * @return Upon successful return, these functions return the
     *         number of characters printed (excluding the null byte
     *         used to end output to strings).
     *         If an output error is encountered, a negative value is returned.
     */
    static int printf_status_packet(const InnoStatusPacket &pkt, char *buffer,
                                    size_t buffer_size);

    /*
     * @brief init Raw4UdpHeader by net buffer.
     * @param buffer
     * @param buffer_size
     * @param header
     * @return true: successful, false: failed.
     */
    static bool raw4_header_from_net(const char *buffer, size_t buffer_size,
                                     Raw4UdpHeader *header);

    /*
     * @brief fill net buffer by init Raw4UdpHeader.
     * @param header
     * @param buffer
     * @param buffer_size
     * @return true: successful, false: failed.
     */
    static bool raw4_header_to_net(const Raw4UdpHeader &header, char *buffer,
                                   size_t buffer_size);

    /*
     * @brief Get the number of points in the pkt, have to iterate each
     *        block for INNO_ITEM_TYPE_SPHERE_POINTCLOUD pkt.
     * @param pkt DataPacket
     * @return number of points in the pkt
     */
    static inline uint32_t get_points_count(const InnoDataPacket &pkt) {
      if (pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
        return pkt.item_number;
      } else if (pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
        uint32_t item_count = 0;
        uint32_t dummy_count = 0;
#define ADD_FN(ctx, p, b, pt, full_angles, ch, m) \
        do {                                      \
          if (pt.radius) {                        \
            item_count++;                         \
          }                                       \
        } while (0)

        ITERARATE_INNO_DATA_PACKET_CPOINTS(ADD_FN, NULL,
                                           &pkt, dummy_count);
        return item_count;
      } else {
        inno_log_verify(false, "invalid type %u", pkt.type);
      }
    }

    static inline uint64_t get_points_count_2nd_return(
        const InnoDataPacket &pkt) {
      if (pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
        uint64_t cnt = 0;
        for (int i = 0; i < pkt.item_number; ++i) {
          cnt += pkt.xyz_points[i].is_2nd_return;
        }
        return cnt;
      } else {
        inno_log_verify(false, "invalid type %u", pkt.type);
      }
    }

    /*
     * @brief Get max number of points in the pkt. For
     *        INNO_ITEM_TYPE_SPHERE_POINTCLOUD pkt it is calculated
     *        based on number of blocks and number of returns.
     * @param pkt DataPacket
     * @return max number of points in the pkt
     */
    static inline uint32_t get_max_points_count(const InnoDataPacket &pkt) {
      if (pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
        return pkt.item_number;
      } else if (pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
        return pkt.item_number * kInnoChannelNumber *
            (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE ?
             1 : 2);
      } else {
        inno_log_verify(false, "invalid type %u", pkt.type);
      }
    }
  };

  class InnoPacketReader {
   public:
    /*
     * @brief Read one data packet or message packet or status packet
     *        from fd
     * @param data_packet Pointer to the buffer that the received data
     *                    packet will be written to
     * @param data_len The maximum len the data_packet can be, if
     *                 the size is not enough, the function will return -3.
     *                 If one data_packet is received, data_len will be
     *                 set to the size of the packet, otherwise set to 0.
     * @param message_packet Pointer to the buffer that the received
     *                       message packet will be written to
     * @param message_len The maximum len the message_packet can be, if
     *                 the size is not enough, the function will return -3.
     *                 If one message_packet is received, message_len will be
     *                 set to the size of the packet, otherwise set to 0.
     * @param status_packet Pointer to the buffer that the received status
     *                      packet will be written to.
     * @param status_len The maximum len the status_packet can be, if
     *                   the size is not enough, the function will return -3
     *                   If one status_packet is received, status_len will 
     *                   ve set to the size of the packet, otherwise set to 0.
     * @param is_file The fd is for a file, not a network socket.
     *
     * @return 0 means success
     *         -1 means file ends or connection disconnected before
     *            enough data was read
     *         -2 recieved data is not valid
     *         -3 the buffers are too small
     */
    static int read_packet(int fd,
                           InnoDataPacket *data_packet,
                           size_t *data_len,
                           InnoDataPacket *message_packet,
                           size_t *message_len,
                           InnoStatusPacket *status_packet,
                           size_t *status_len,
                           bool is_file);

    static uint32_t calculate_packet_crc32(const InnoCommonHeader *header);

    static void set_packet_crc32(InnoCommonHeader *header);

    static bool verify_packet_crc32(const InnoCommonHeader *header);

    static uint32_t calculate_http_crc32(const char* buffer,
                                         uint32_t length, bool append = false);

    static int verify_http_crc32(const char* buffer, const char* url);
  };
};  // extern "C"

class InnoSummaryPackage {
 public:
  /*
  * @brief Collect statistics about Miss frames and Miss sub frames 
           based on data stream.
    @param pkt the stream of innoDataPacket
    @return 0 means success
            -1 means miss sub frame
            -2 means miss frame
  */
  InnoSummaryPackage() {
    current_frame_ = 0;
    expect_frame_ = 0;
    current_sub_frame_ = 0;
    expect_sub_frame_ = -1;
    next_new_frame_ = false;
    frame_counter_ = 0;
    miss_frame_counter_ = 0;
    miss_sub_frame_gap_counter_ = 0;
    miss_sub_frame_last_one_counter_ = 0;
    miss_sub_frame_except_last_one_counter_ = 0;
    empty_sub_frame_counter_ = 0;
  }
  ~InnoSummaryPackage() {}
  int summary_data_package(const InnoDataPacket &pkt);
  uint64_t get_miss_frame_count(void) {
    return miss_frame_counter_;
  }
  uint64_t get_miss_sub_frame_gap_count(void) {
    return  miss_sub_frame_gap_counter_;
  }
  uint64_t get_frame_count(void) {
    if (frame_counter_ > 0) {
      return frame_counter_;
    } else {
      return 0;
    }
  }
  uint64_t get_empty_sub_frame_count(void) {
    return empty_sub_frame_counter_;
  }
  uint64_t get_miss_sub_frame_last_one_count(void) {
    return miss_sub_frame_last_one_counter_;
  }
  uint64_t get_empty_sub_frame_except_last_one_count(void) {
    return miss_sub_frame_except_last_one_counter_;
  }

 private:
  int64_t current_frame_;
  int64_t expect_frame_;
  uint32_t current_sub_frame_;
  int32_t expect_sub_frame_;
  bool next_new_frame_;
  uint64_t frame_counter_;
  uint64_t miss_frame_counter_;
  uint64_t miss_sub_frame_gap_counter_;
  uint64_t miss_sub_frame_last_one_counter_;
  uint64_t miss_sub_frame_except_last_one_counter_;
  uint64_t empty_sub_frame_counter_;
};

#endif  // SDK_COMMON_INNO_LIDAR_PACKET_UTILS_H_
