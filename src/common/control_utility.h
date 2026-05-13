/**
 * Copyright @ 2021 - 2023 JIDU AUTO CO.,LTD.
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are NOT permitted except as agreed by
 * JIDU AUTO CO.,LTD.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once

#include <string>

#include "pnc_common/src/common/pnc_logger.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {
/**
 * @brief provide utility functions.
 */
#define check_proto(a, b)                                   \
    if (!a.has_##b()) {                                     \
        LOG_ERROR("PB invalid! [%s] has NO [%s]!", #a, #b); \
        return false;                                       \
    }

#define check_proto_nan(a)                         \
    if (std::isnan(a)) {                           \
        LOG_ERROR("PB invalid! [%s] is nan!", #a); \
        return false;                              \
    }

constexpr const char* ESTOP_NORMAL = "Normal";
constexpr const char* ESTOP_PLANLOST = "Planning lost";
constexpr const char* ESTOP_LOCALLOST = "Localization lost";
constexpr const char* ESTOP_CANLOST = "Canbus lost";
constexpr const char* ESTOP_PLANESTOP = "Planning request estop";
constexpr const char* ESTOP_CONTROLFAIL = "Control failed estop";
constexpr const char* ESTOP_CONTROLSAFETY = "Control safety estop";
constexpr const char* ESTOP_SPEEDPROTECTION = "Localization and Chassis speed protection estop";
constexpr const char* ESTOP_PerceptionLaneLOST = "Perception lane lost";

struct EstopInfo {
    bool estop = false;
    bool hard_estop = false;
    std::string stop_reason = ESTOP_NORMAL;
};

}  // namespace control
}  // namespace jiduauto
