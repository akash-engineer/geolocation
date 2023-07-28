/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "utils/types_consts.h"

#include "utils/log.h"

namespace innovusion {
const double InnoConsts::kAirRefractiveIndex = 1.0003;
const double InnoConsts::kLightTraveMeterPerSubNs =
      InnoConsts::kSpeedOfLight / InnoConsts::kAirRefractiveIndex /
    InnoConsts::kNsInSecond /
    InnoConsts::kSubNsInNs / 2;
}  // namespace innovusion
