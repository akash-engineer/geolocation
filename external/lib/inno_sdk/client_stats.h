/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CLIENT_CLIENT_STATS_H_
#define SDK_CLIENT_CLIENT_STATS_H_

#include <limits.h>

#include "sdk_common/resource_stats.h"

namespace innovusion {
class InnoLidarClient;

class ClientStats : public ResourceStats {
 public:
  explicit ClientStats(InnoLidarClient *l);
  ~ClientStats() {
  }

 public:
  void get_extra_info_(char *buf, size_t buf_size,
                       double time_diff);

 private:
  InnoLidarClient *lidar_client_;
};

}  // namespace innovusion

#endif  // SDK_CLIENT_CLIENT_STATS_H_
