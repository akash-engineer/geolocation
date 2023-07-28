/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "./inno_lidar_packet_utils.h"

#include <algorithm>
#include <cmath>

#include "./nps_adjustment.h"
#include "../utils/inno_lidar_log.h"
#include "../utils/math_tables.h"
#include "../utils/net_manager.h"
#include "../utils/utils.h"

int InnoDataPacketUtils::init_ = InnoDataPacketUtils::init_f();
int InnoDataPacketUtils::v_angle_offset_[kInnoChannelNumber];
int8_t InnoDataPacketUtils::nps_adjustment_[kVTableSize_][kHTableSize_][kInnoChannelNumber][kXZSize_];  // NOLINT
const double InnoDataPacketUtils::kAdjustmentUnitInMeter_ = 0.0025;

int InnoPacketReader::read_packet(int fd,
                                  InnoDataPacket *data_packet,
                                  size_t *data_len,
                                  InnoDataPacket *message_packet,
                                  size_t *message_len,
                                  InnoStatusPacket *status_packet,
                                  size_t *status_len,
                                  bool is_file) {
  union {
    InnoCommonHeader header;
    char a_[0];
    int d_[0];
  };
  inno_log_verify(data_packet && data_len, "NULL pointer");
  inno_log_verify(message_packet && message_len, "NULL pointer");
  inno_log_verify(status_packet && status_len, "NULL pointer");
  inno_log_verify(*data_len >= sizeof(InnoDataPacket),
                  "%" PRI_SIZEU " too small", *data_len);
  inno_log_verify(*message_len >= sizeof(InnoDataPacket),
                  "%" PRI_SIZEU " too small", *message_len);
  inno_log_verify(*status_len >= sizeof(InnoStatusPacket),
                  "%" PRI_SIZEU " too small", *status_len);
  int flag = is_file ? -1 : 0;

  int ret = innovusion::NetManager::recv_full_buffer(
      fd, a_,
      sizeof(InnoCommonHeader), flag);
  if (ret != ssize_t(sizeof(InnoCommonHeader))) {
    inno_log_info("can not read version header, read return %d", ret);
    return 0;
  }
  if (header.version.magic_number == kInnoMagicNumberStatusPacket) {
    status_packet->common = header;
    int to_read = sizeof(InnoStatusPacket) - sizeof(InnoCommonHeader);
    ret = innovusion::NetManager::recv_full_buffer(
        fd, reinterpret_cast<char *>(status_packet) + sizeof(InnoCommonHeader),
        to_read, flag);
    if (ret < to_read) {
      inno_log_warning("can not read version header, read return %d", ret);
      return -1;
    } else if (sizeof(InnoStatusPacket) != header.size) {
      inno_log_warning("bad data header size, read return %d vs %u",
                       ret, header.size);
      return -2;
    } else {
      *data_len = 0;
      *message_len = 0;
      *status_len = sizeof(InnoStatusPacket);
      return header.size;
    }
  } else if (header.version.magic_number == kInnoMagicNumberDataPacket) {
    InnoDataPacket dp;
    dp.common = header;
    int to_read = sizeof(InnoDataPacket) - sizeof(InnoCommonHeader);
    ret = innovusion::NetManager::recv_full_buffer(
        fd,
        reinterpret_cast<char *>(&dp) + sizeof(InnoCommonHeader),
        to_read, flag);
    if (ret != to_read) {
      inno_log_warning("can not read data header, read return %d %d",
                       ret, to_read);
      return -1;
    }
    if (header.size < sizeof(InnoDataPacket)) {
      inno_log_warning("bad data header size %u vs %" PRI_SIZEU "",
                       header.size, sizeof(InnoDataPacket));
      /* FIXME array subscript is above array bounds [-Werror=array-bounds]
      inno_log_info("%x %x %x %x %x %x",
                    d_[0], d_[1], d_[2], d_[3], d_[4], d_[5]); */
      return -2;
    }
    char *read_pt = NULL;
    *status_len = 0;
    if (dp.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD ||
        dp.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
      read_pt = reinterpret_cast<char *>(data_packet + 1);
      *message_len = 0;
      if (*data_len < header.size) {
        inno_log_warning("not enough buffer message: %" PRI_SIZEU " %u",
                         *data_len, header.size);
        return -3;
      }
      *data_len = header.size;
      *data_packet = dp;
    } else if (dp.type == INNO_ITEM_TYPE_MESSAGE ||
               dp.type == INNO_ITEM_TYPE_MESSAGE_LOG) {
      read_pt = reinterpret_cast<char *>(message_packet + 1);
      *data_len = 0;
      if (*message_len < header.size) {
        inno_log_warning("not enough buffer message: %" PRI_SIZEU " %u",
                         *data_len, header.size);
        return -3;
      }
      *message_len = header.size;
      *message_packet = dp;
    } else {
      inno_log_warning("invalid data type %d",
                       dp.type);
      return -2;
    }
    to_read = header.size - sizeof(InnoDataPacket);

    ret = innovusion::NetManager::recv_full_buffer(fd, read_pt,
                                                   to_read, flag);
    if (ret < to_read) {
      inno_log_warning("cannot read data, read return %d/%d/%u",
                       ret, to_read, header.size);
      return -1;
    } else {
      return header.size;
    }
  } else {
    inno_log_warning("invalid magic number 0x%hx",
                     header.version.magic_number);
    return -2;
  }
}

uint32_t InnoPacketReader::calculate_packet_crc32(
    const InnoCommonHeader *header) {
  uint32_t crc = innovusion::InnoUtils::crc32_start();
  crc = innovusion::InnoUtils::crc32_do(crc, &header->version,
                                        sizeof(header->version));
  size_t off = offsetof(struct InnoCommonHeader, size);
  crc = innovusion::InnoUtils::crc32_do(crc, &header->size,
                                        header->size - off);
  return innovusion::InnoUtils::crc32_end(crc);
}

void InnoPacketReader::set_packet_crc32(InnoCommonHeader *header) {
  header->checksum = calculate_packet_crc32(header);
}

bool InnoPacketReader::verify_packet_crc32(const InnoCommonHeader *header) {
  uint32_t checksum = calculate_packet_crc32(header);
  if (header->checksum != checksum) {
    inno_log_warning("checksum mismatch 0x%x vs 0x%x",
                     header->checksum, checksum);
    return false;
  } else {
    return true;
  }
}

uint32_t InnoPacketReader::calculate_http_crc32(const char* buffer,
                                                uint32_t length,
                                                bool append) {
  return innovusion::InnoUtils::calculate_http_crc32(buffer,
                                                     length,
                                                     append);
}

int InnoPacketReader::verify_http_crc32(const char* buffer,
                                        const char* url) {
  return innovusion::InnoUtils::verify_http_crc32(buffer,
                                                  url);
}


int InnoDataPacketUtils::init_f(void) {
  for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
    v_angle_offset_[ich] = ich * kInnoVAngleDiffBase;
  }

  // init the nps_adjustment_
  size_t input_size = (kVTableEffeHalfSize_ * 2 + 1) *
                      (kHTableEffeHalfSize_ * 2 + 1) *
                      2 * kInnoChannelNumber;
  inno_log_verify(sizeof(kInnoPs2Nps) ==
                  input_size * sizeof(double), "kInnoPs2Nps");

  inno_log_verify(kVTableSize_ >= kVTableEffeHalfSize_ * 2 + 1,
                  "kVTableSize_");
  inno_log_verify(kHTableSize_ >= kHTableEffeHalfSize_ * 2 + 1,
                  "kHTableSize_");
  memset(nps_adjustment_, 0, sizeof(nps_adjustment_));
  static double k_max[2] = {-100, -100};
  static double k_min[2] = {100, 100};
  for (uint32_t v = 0; v < kVTableEffeHalfSize_ * 2 + 1; v++) {
    for (uint32_t h = 0; h < kHTableEffeHalfSize_ * 2 + 1; h++) {
      for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
        for (uint32_t xz = 0; xz < kXZSize_; xz++) {
          double k = kInnoPs2Nps[xz][ich][v][h];
          double u = k / kAdjustmentUnitInMeter_;
          double q = std::floor(u + 0.5);
          nps_adjustment_[v][h][ich][xz] = q;
          // inno_log_debug("k=%f q=%f %d",
          //                k, q, nps_adjustment_[v][h][ich][xz]);
          k_max[xz] = std::max(k_max[xz], k);
          k_min[xz] = std::min(k_min[xz], k);
        }
      }
    }
  }
  // nps: x_adj=[-0.028944, -0.010921] range=0.018023, z_adj=[0.027061,0.037247] range=0.010186 NOLINT
  /*
  inno_log_info("nps: x_adj=[%f, %f] range=%f, "
                "z_adj=[%f,%f] range=%f",
                k_min[0], k_max[0],
                k_max[0] - k_min[0],
                k_min[1], k_max[1],
                k_max[1] - k_min[1]);
  */
  return 0;
}

inline void InnoDataPacketUtils::lookup_xz_adjustment_(
    const InnoBlockAngles &angles,
    uint32_t ch,
    double *x, double *z) {
  uint32_t v = angles.v_angle / 512;
  uint32_t h = angles.h_angle / 512;
  v += kVTableEffeHalfSize_;
  h += kHTableEffeHalfSize_;
  // avoid index out-of-bound
  v = v & (kVTableSize_ - 1);
  h = h & (kHTableSize_ - 1);
  int8_t *addr_x = &nps_adjustment_[v][h][ch][0];
  int8_t *addr_z = addr_x + 1;
  *x = *addr_x * kAdjustmentUnitInMeter_;
  *z = *addr_z * kAdjustmentUnitInMeter_;
  return;
}

void InnoDataPacketUtils::get_xyzr_meter(
    const InnoBlockAngles angles,
    const uint32_t radius_unit,
    const uint32_t channel,
    InnoXyzrD *result) {
  result->radius = radius_unit * kMeterPerInnoDistanceUnit;
  double t;
  if (angles.v_angle >= 0) {
    t = result->radius *
              innovusion::MathTables::lookup_cos_table_in_unit(angles.v_angle);
    result->x = result->radius *
              innovusion::MathTables::lookup_sin_table_in_unit(angles.v_angle);
  } else {
    t = result->radius *
              innovusion::MathTables::lookup_cos_table_in_unit(-angles.v_angle);
    result->x = -result->radius *
              innovusion::MathTables::lookup_sin_table_in_unit(-angles.v_angle);
  }
  if (angles.h_angle >= 0) {
    result->y = t *
              innovusion::MathTables::lookup_sin_table_in_unit(angles.h_angle);
    result->z = t *
              innovusion::MathTables::lookup_cos_table_in_unit(angles.h_angle);
  } else {
    result->y = -t *
              innovusion::MathTables::lookup_sin_table_in_unit(-angles.h_angle);
    result->z = t *
              innovusion::MathTables::lookup_cos_table_in_unit(-angles.h_angle);
  }
  double x_adj, z_adj;
  lookup_xz_adjustment_(angles, channel, &x_adj, &z_adj);

  // inno_log_debug("adjust %f %f", x_adj, z_adj);
  result->x += x_adj;
  result->z += z_adj;
  return;
}

InnoDataPacket *InnoDataPacketUtils::convert_to_xyz_pointcloud_malloced(
    const InnoDataPacket &src, RingIdConverterInterface *ring_id_converter) {
  // 1. calculate max size and allocate new data packet
  size_t new_pkt_size = get_data_packet_size(
      INNO_ITEM_TYPE_XYZ_POINTCLOUD,
      InnoDataPacketUtils::get_max_points_count(src),
      INNO_MULTIPLE_RETURN_MODE_NONE);

  InnoDataPacket *new_pkt =
      reinterpret_cast<InnoDataPacket *>(malloc(new_pkt_size));
  if (!new_pkt) {
    return NULL;
  }

  // 2. convert INNO_ITEM_TYPE_SPHERE_POINTCLOUD to
  //    INNO_ITEM_TYPE_XYZ_POINTCLOUD
  bool ret = convert_to_xyz_pointcloud(src,
                                       new_pkt,
                                       new_pkt_size,
                                       false,
                                       ring_id_converter);
  if (ret) {
    return new_pkt;
  } else {
    free(new_pkt);
    return NULL;
  }
}

bool InnoDataPacketUtils::convert_to_xyz_pointcloud(
    const InnoDataPacket &src,
    InnoDataPacket *dest,
    size_t dest_size,
    bool append,
    RingIdConverterInterface *ring_id_converter) {
  if (src.type != INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
    inno_log_warning("invalid type %u", src.type);
    return false;
  }

  if (!check_data_packet(src, 0)) {
    inno_log_warning("invalid src datapacket");
    return false;
  }

  uint32_t item_count = 0;
  uint32_t dummy_count = 0;
  item_count = get_points_count(src);

  if (append) {
    if (!check_data_packet(*dest, dest_size)) {
      inno_log_warning("invalid dest datapacket");
      return false;
    }
    item_count += dest->item_number;
  }

  size_t required_size = 0;
  uint16_t time_adjust_10us = 0;
  if (!append) {
    required_size = sizeof(InnoDataPacket);
    if (required_size > dest_size) {
      inno_log_warning("not enough size %" PRI_SIZEU " %" PRI_SIZEU "",
                       required_size, dest_size);
      return false;
    }
    memcpy(dest, &src, sizeof(InnoDataPacket));
    dest->type = INNO_ITEM_TYPE_XYZ_POINTCLOUD;
    dest->item_size = sizeof(InnoXyzPoint);
    dest->item_number = 0;
  } else {
    required_size = src.common.size;
    if (src.common.ts_start_us < dest->common.ts_start_us) {
      inno_log_warning("cannot merge earlier packet %f %f",
                       src.common.ts_start_us,
                       dest->common.ts_start_us);
      return false;
    }
    time_adjust_10us = (src.common.ts_start_us -
                        dest->common.ts_start_us) / 10;
  }

  {
#define CONVERT_FN(ctx, p, blk, pt, full_angles, ch, m)    \
    do {                                                   \
      if (pt.radius > 0) {                                 \
        InnoXyzPoint &ipt = dest->xyz_points[dest->item_number]; \
        required_size += sizeof(InnoXyzPoint);             \
        if (required_size > dest_size) {                   \
            inno_log_warning("not enough size %" PRI_SIZEU " %" \
                             PRI_SIZEU "",                      \
                             required_size, dest_size);    \
            return false;                                  \
        }                                                  \
        get_xyz_point(                                     \
            blk.header, pt, full_angles.angles[ch], ch,    \
            &ipt);                                         \
        ipt.is_2nd_return = pt.is_2nd_return;              \
        ipt.ts_10us += time_adjust_10us;                   \
        if (ring_id_converter) {                           \
          ipt.ring_id = ring_id_converter->get_ring_id(    \
                                  (InnoLidarMode)p.common.lidar_mode, \
                                  p.scanner_direction,     \
                                  blk.header.scan_id, ch); \
        }                                                  \
        dest->item_number++;                               \
      }                                                    \
    } while (0)

    ITERARATE_INNO_DATA_PACKET_CPOINTS(CONVERT_FN, NULL,
                                       &src, dummy_count);
  }
  inno_log_verify(dest->item_number == item_count,
                  "item number %u vs %u",
                  dest->item_number, item_count);
  dest->common.size = required_size;

  InnoPacketReader::set_packet_crc32(&dest->common);

  if (!check_data_packet(*dest, dest_size)) {
    inno_log_verify(false, "invalid dest datapacket");
    return false;
  }

  return true;
}

bool InnoDataPacketUtils::check_data_packet(const InnoDataPacket &pkt,
                                            size_t size) {
  if (pkt.common.version.magic_number != kInnoMagicNumberDataPacket) {
    inno_log_warning("bad magic %x", pkt.common.version.magic_number);
    return false;
  }
  if (size && pkt.common.size > size) {
    inno_log_warning("bad size %" PRI_SIZEU " %u",
                     size, pkt.common.size);
    return false;
  }
  if (pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
    if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
      if (pkt.item_size != sizeof(InnoBlock1)) {
        inno_log_warning("bad block1 item size %u", pkt.item_size);
        return false;
      }
    } else if (pkt.multi_return_mode ==
               INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
               pkt.multi_return_mode ==
               INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
      if (pkt.item_size != sizeof(InnoBlock2)) {
        inno_log_warning("bad block2 item size %u", pkt.item_size);
        return false;
      }
    } else {
      inno_log_warning("bad return_mode %u", pkt.multi_return_mode);
      return false;
    }
  } else if (pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
    if (pkt.item_size != sizeof(InnoXyzPoint)) {
      inno_log_warning("bad InnoXyzPoint item size %u", pkt.item_size);
      return false;
    }
  }

  if (pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD ||
      pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) {
    size_t s = get_data_packet_size(
        InnoItemType(pkt.type),
        pkt.item_number,
        InnoMultipleReturnMode(pkt.multi_return_mode));
    if (pkt.common.size != s) {
      inno_log_warning("bad size %" PRI_SIZEU " %u", s, pkt.common.size);
      return false;
    }
    if (!InnoPacketReader::verify_packet_crc32(&pkt.common)) {
      inno_log_warning("crc32 mismatch for data packet");
      return false;
    }
    return true;
  } else if (pkt.type == INNO_ITEM_TYPE_MESSAGE ||
             pkt.type == INNO_ITEM_TYPE_MESSAGE_LOG) {
    if (pkt.item_number != 1) {
      inno_log_warning("bad item_number %u", pkt.item_number);
      return false;
    }
    if ((size_t)pkt.item_size != pkt.messages[0].size ||
        pkt.item_size <= sizeof(InnoMessage)) {
      inno_log_warning("bad message size %u %u",
                       pkt.item_size, pkt.messages[0].size);
      return false;
    }
    if (pkt.common.size !=
        pkt.item_size + sizeof(InnoDataPacket)) {
      inno_log_warning("bad message size %u, %" PRI_SIZEU "",
                       pkt.common.size,
                       pkt.item_size + sizeof(InnoDataPacket));
      return false;
    }
    if (!InnoPacketReader::verify_packet_crc32(&pkt.common)) {
      inno_log_warning("crc32 mismatch for message packet");
      return false;
    }
    return true;
  } else {
    inno_log_warning("bad type %u", pkt.type);
    return false;
  }
}

bool InnoDataPacketUtils::check_status_packet(const InnoStatusPacket &pkt,
                                              size_t size) {
  if (pkt.common.version.magic_number != kInnoMagicNumberStatusPacket) {
    inno_log_warning("bad magic %x", pkt.common.version.magic_number);
    return false;
  }
  if (size && pkt.common.size > size) {
    inno_log_warning("bad size status packet %" PRI_SIZEU " %u",
                     size, pkt.common.size);
    return false;
  }
  if (pkt.common.size != sizeof(pkt)) {
    inno_log_warning("bad size status packet %u %" PRI_SIZEU "",
                     pkt.common.size, sizeof(pkt));
    return false;
  }

  if (!InnoPacketReader::verify_packet_crc32(&pkt.common)) {
    inno_log_warning("crc32 mismatch for status packet");
    return false;
  }
  return true;
}


int InnoDataPacketUtils::printf_status_packet(const InnoStatusPacket &pkt,
                                               char *buffer,
                                               size_t buffer_size) {
  if (!buffer) {
    return -1;
  }

  // InnoStatusInFaults in_faults;
  // InnoStatusExFaults ex_faults;

  // InnoStatusCounters counters;
  auto &c = pkt.counters;
  int ret_c = snprintf(
      buffer, buffer_size,
      "counters: "
      "point_data_packet_sent=%" PRI_SIZEU " "
      "point_sent=%" PRI_SIZEU " "
      "message_packet_sent=%" PRI_SIZEU " "
      "raw_data_read=%" PRI_SIZEU " "
      "total_frame=%" PRI_SIZEU " "
      "total_polygon_rotation=%" PRI_SIZEU " "
      "total_polygon_facet=%" PRI_SIZEU " "
      "power_up_time_in_second=%u "
      "process_up_time_in_second=%u "
      "lose_ptp_sync=%u "
      "bad_data[4]=%u/%u/%u/%u "
      "data_drop[8]=%u/%u/%u/%u/%u/%u/%u/%u "
      "signals[8]=%u/%u/%u/%u/%u/%u/%u/%u "
      "latency_10us_average[6]=%hu/%hu/%hu/%hu/%hu/%hu "
      "latency_10us_variation[6]=%hu/%hu/%hu/%hu/%hu/%hu "
      "latency_10us_max[6]=%hu/%hu/%hu/%hu/%hu/%hu "
      "big_latency_frame=%u "
      "bad_frame=%u "
      "big_gap_frame=%u "
      "small_gap_frame=%u "
      "cpu_percentage=%hu "
      "mem_percentage=%hu "
      "netstat_rx speed/drop/err=%hukBps/%hu/%hu "
      "netstat_tx speed/drop/err=%hukBps/%hu/%hu "
      "sys_cpu_percentage=%hu/%hu/%hu/%hu "
      "motor[5]=%hu/%hu/%hu/%hu/%hu "
      "galvo[5]=%hu/%hu/%hu/%hu/%hu "
      "in_faults=0x%" PRI_SIZEX " "
      ";",
      c.point_data_packet_sent, c.point_sent, c.message_packet_sent,
      c.raw_data_read, c.total_frame, c.total_polygon_rotation,
      c.total_polygon_facet, c.power_up_time_in_second,
      c.process_up_time_in_second, c.lose_ptp_sync,
      // c.bad_data[4]
      c.bad_data[0], c.bad_data[1], c.bad_data[2], c.bad_data[3],
      // data_drop[8]
      c.data_drop[0], c.data_drop[1], c.data_drop[2], c.data_drop[3],
      c.data_drop[4], c.data_drop[5], c.data_drop[6], c.data_drop[7],
      // signals[8]
      c.in_signals[0], c.in_signals[1], c.in_signals[2], c.in_signals[3],
      c.in_signals[4], c.in_signals[5], c.in_signals[6], c.in_signals[7],
      // latency_10us_average[6]
      c.latency_10us_average[0], c.latency_10us_average[1],
      c.latency_10us_average[2], c.latency_10us_average[3],
      c.latency_10us_average[4], c.latency_10us_average[5],
      // latency_10us_variation[6]
      c.latency_10us_variation[0], c.latency_10us_variation[1],
      c.latency_10us_variation[2], c.latency_10us_variation[3],
      c.latency_10us_variation[4], c.latency_10us_variation[5],
      // latency_10us_max[6]
      c.latency_10us_max[0], c.latency_10us_max[1], c.latency_10us_max[2],
      c.latency_10us_max[3], c.latency_10us_max[4], c.latency_10us_max[5],
      c.big_latency_frame, c.bad_frame, c.big_gap_frame, c.small_gap_frame,
      c.cpu_percentage, c.mem_percentage,
      // netstat
      c.netstat_rx_speed_kBps, c.netstat_rx_drop, c.netstat_rx_err,
      c.netstat_tx_speed_kBps, c.netstat_tx_drop, c.netstat_tx_err,
      // usage of each proc
      c.sys_cpu_percentage[0], c.sys_cpu_percentage[1],
      c.sys_cpu_percentage[2], c.sys_cpu_percentage[3],
      // motor[5]
      c.motor[0], c.motor[1], c.motor[2], c.motor[3], c.motor[4],
      // galvo[5]
      c.galvo[0], c.galvo[1], c.galvo[2], c.galvo[3], c.galvo[4],
      // in_faults
      pkt.in_faults.faults);

  if (ret_c < 0) {    // error
    return ret_c;
  }

  if (static_cast<size_t>(ret_c) >= buffer_size) {
    buffer[buffer_size - 1] = 0;
    return buffer_size - 1;
  }

  // InnoStatusSensorReadings sensor_readings;
  const InnoStatusSensorReadings &s = pkt.sensor_readings;
  int ret_s = snprintf(
      buffer + ret_c, buffer_size - ret_c,
      "sensor_readings: "
      "t_fpga=%hd t_laser=%hd "
      "t_adc=%hd t_board=%hd "
      "t_det=%hd/%hd/%hd/%hd "
      "t_other=%hd/%hd/%hd "
      "heater_ma=%hu "
      "m_rpm=%u g_fpm=%u "
      "m_total=%" PRI_SIZEU " g_total=%" PRI_SIZEU " "
      "moisture=%hu/%hu "
      "window_blockage=%hu/%hu "
      "motor=%hu/%hu/%hu/%hu/%hu/%hu "
      "galvo=%hu/%hu/%hu/%hu/%hu/%hu "
      "laser=%hu/%hu/%hu/%hu/%hu/%hu "
      "galvo_client=0x%hx/%hu "
      "\n",
      s.temperature_fpga_10th_c, s.temperature_laser_10th_c,
      s.temperature_adc_10th_c, s.temperature_board_10th_c,
      // temperature_det_10th_c
      s.temperature_det_10th_c[0], s.temperature_det_10th_c[1],
      s.temperature_det_10th_c[2], s.temperature_det_10th_c[3],
      // temperature_other_10th_c
      s.temperature_other_10th_c[0], s.temperature_other_10th_c[1],
      s.temperature_other_10th_c[2],
      //
      s.heater_current_ma, s.motor_rpm_1000th, s.galvo_fpm_1000th,
      s.motor_rotation_total, s.galvo_round_total,
      // moisture_index
      s.moisture_index[0], s.moisture_index[1],
      // window_blockage_index
      s.window_blockage_index[0], s.window_blockage_index[1],
      // motor
      s.motor[0], s.motor[1], s.motor[2], s.motor[3], s.motor[4], s.motor[5],
      // galvo
      s.galvo[0], s.galvo[1], s.galvo[2], s.galvo[3], s.galvo[4], s.galvo[5],
      // laser
      s.laser[0], s.laser[1], s.laser[2], s.laser[3], s.laser[4], s.laser[5],
      // galvo mirror offset check in client sdk
      s.galvo_status_client, s.galvo_offset_angle_client);

  if (ret_s < 0) {    // error
    return ret_s;
  }

  if (static_cast<size_t>(ret_c + ret_s) >= buffer_size) {
    buffer[buffer_size - 1] = 0;
    return buffer_size - 1;
  }

  return ret_c + ret_s;
}

bool InnoDataPacketUtils::raw4_header_from_net(const char *buffer,
                                               size_t buffer_size,
                                               Raw4UdpHeader *header) {
  if (buffer_size < Raw4UdpHeader::kHeaderSize) {
    return false;
  }

  if (!buffer || !header) {
    return false;
  }

  header->idx = ntohl(*(reinterpret_cast<const uint32_t *>(buffer)));
  header->field_type = buffer[Raw4UdpHeader::kFieldTypeOffset];
  header->field_sequence_id = ntohl(*(reinterpret_cast<const uint32_t *>(
      buffer + Raw4UdpHeader::kFieldSeqIdOffset)));
  header->flag = buffer[Raw4UdpHeader::kFlagOffset];

  return true;
}

bool InnoDataPacketUtils::raw4_header_to_net(const Raw4UdpHeader &header,
                                             char *buffer, size_t buffer_size) {
  if (!buffer || buffer_size < Raw4UdpHeader::kHeaderSize) {
    return false;
  }

  *(reinterpret_cast<uint32_t *>(buffer)) = htonl(header.idx);
  buffer[Raw4UdpHeader::kFieldTypeOffset] = header.field_type;
  *(reinterpret_cast<uint32_t *>(buffer + Raw4UdpHeader::kFieldSeqIdOffset)) =
      htonl(header.field_sequence_id);
  buffer[Raw4UdpHeader::kFlagOffset] = header.flag;

  return true;
}

int InnoSummaryPackage::summary_data_package
                        (const InnoDataPacket &pkt) {
  int result = 0;
  inno_log_trace("pkt.idx=%" PRI_SIZEU ", pkt.sub_idx=%d, last=%d",
                 pkt.idx, pkt.sub_idx,
                 pkt.is_last_sub_frame);
  // new frame
  if (current_frame_ != (int64_t)pkt.idx) {
    // last frame don't receive is_last_sub_frame flag
    if (!next_new_frame_ && expect_frame_ > 0) {
      inno_log_warning("Miss frame %" PRI_SIZED " 's last one packet.",
                        current_frame_);
      miss_sub_frame_last_one_counter_++;
      miss_sub_frame_gap_counter_++;
      result = -1;
    }
    // check whether is expected frame ID
    if (expect_frame_ != (int64_t)pkt.idx && expect_frame_ > 0) {
      inno_log_warning("The expected frame is %" PRI_SIZED ", "
      "but the current frame is %" PRI_SIZEU "",
      expect_frame_, pkt.idx);
      if ((int64_t)pkt.idx > expect_frame_) {
        miss_frame_counter_ += (pkt.idx - expect_frame_);
        result = -2;
      }
    }
    current_frame_ = (int64_t)pkt.idx;
    expect_frame_ = current_frame_ + 1;
    frame_counter_++;
    next_new_frame_ = false;
  }

  current_sub_frame_ = pkt.sub_idx;
  // check whether is expect sub frame
  if (expect_sub_frame_ != (int32_t)current_sub_frame_
       && result == 0 && expect_sub_frame_ > 0) {
    inno_log_warning("UDP sub frame miss. The current frame is %"
                     PRI_SIZEU ", "
    "expect_sub_frame = %d, but current_sub_frame = %u",
    pkt.idx, expect_sub_frame_, current_sub_frame_);
    miss_sub_frame_gap_counter_++;
    if ((int32_t)current_sub_frame_ > expect_sub_frame_) {
      miss_sub_frame_except_last_one_counter_ +=
                    current_sub_frame_ - expect_sub_frame_;
      inno_log_warning("Miss sub frame id from %d to %u",
                        expect_sub_frame_, current_sub_frame_ - 1);
      result = -1;
    }
  }
  expect_sub_frame_ = current_sub_frame_ + 1;
  if (pkt.is_last_sub_frame) {
    // next frame is new frame
    next_new_frame_ = true;
    expect_sub_frame_ = 0;
  }
  if (pkt.item_number == 0) {
    inno_log_info("Empty sub frame. Current_frame_ is %" PRI_SIZEU ", "
      "current_sub_frame = %u",
      pkt.idx, current_sub_frame_);
    empty_sub_frame_counter_++;
  }

  return result;
}
