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

// noise generator Module
// Refer to https://wiki.jiduauto.com/pages/viewpage.action?pageId=560802854

#pragma once

#include "control/src/common/control_message_stream.h"
#include "pnc_control_config.pb.h"
#include "pnc_control_preprocess_conf.pb.h"
#include "pnc_localization_estimate.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

struct LocalizationInformation {
    double pure_x{0.0};
    double pure_y{0.0};
    double pure_yaw{0.0};
    double pure_roll{0.0};
    double pure_pitch{0.0};
    double noised_x{0.0};
    double noised_y{0.0};
    double noised_yaw{0.0};
};
class NoiseGenerator {
public:
    /**
     * @brief Default constructor.
     */
    NoiseGenerator() = default;

    /**
     * @brief destructor.
     */
    ~NoiseGenerator() = default;

    /**
     * @brief generate noise into localization info, include position xy, orientation yaw
     */
    bool AddNoiseToLocalization(pnc::localization::LocalizationEstimate* const localization_ptr);

    /**
     * @brief
     */
    void SetDebugInfo(pnc::control::ControlDebug* const control_debug);

    bool Reset();

    bool Init(const pnc::control::ControlConfig& control_conf);

    /**
     *
     *
     */

private:
    pnc::control::NoiseGeneratorConf noise_generator_conf_;
    LocalizationInformation localzation_info_;

    /**
     * @brief update localization info
     */
    bool UpdateLocalizationInfo(const pnc::localization::LocalizationEstimate* const localization_ptr);

    void GenerateNoisedXY(LocalizationInformation* const localzation_info_ptr);

    void GenerateNoisedYaw(LocalizationInformation* const localzation_info_ptr);

    double GenerateGaussianNoise(const double mean, const double sigma);
};
}  // namespace control
}  // namespace jiduauto
