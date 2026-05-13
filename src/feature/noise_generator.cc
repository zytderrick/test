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

#include "control/src/feature/noise_generator.h"

#include <random>

#include "control/src/common/control_gflags.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"

namespace jiduauto {
namespace control {

bool NoiseGenerator::Init(const pnc::control::ControlConfig& control_conf) {
    if (FLAGS_enable_control_info_print) {
        LOG_INFO("NoiseGenerator is initializing");
    }
    noise_generator_conf_ = control_conf.control_preprocess_conf().noise_generator_conf();
    localzation_info_.pure_x = 0.0;
    localzation_info_.pure_y = 0.0;
    localzation_info_.pure_yaw = 0.0;
    return true;
}

bool NoiseGenerator::Reset() { return true; }

bool NoiseGenerator::AddNoiseToLocalization(pnc::localization::LocalizationEstimate* const localization_ptr) {
    if (localization_ptr == nullptr) {
        LOG_WARN("NOISE:localization_ptr is nullptr or no header");
        return false;
    }

    if (!UpdateLocalizationInfo(localization_ptr)) {
        LOG_WARN("NOISE:UpdateLocalizationInfo failed");
        return false;
    }

    if (FLAGS_enable_localization_noise_xy) {
        // add noise to position xy
        GenerateNoisedXY(&localzation_info_);
        localization_ptr->mutable_loc_pose()->mutable_pose()->mutable_position()->set_x(localzation_info_.noised_x);
        localization_ptr->mutable_loc_pose()->mutable_pose()->mutable_position()->set_y(localzation_info_.noised_y);
    }
    if (FLAGS_enable_localization_noise_heading) {
        // add noise to orientation yaw
        GenerateNoisedYaw(&localzation_info_);
        jiduauto::pnc::EulerAnglesZXYf q_noised(localzation_info_.pure_roll, localzation_info_.pure_pitch,
                                                localzation_info_.noised_yaw);
        q_noised.Normalize();
        auto orientation_noised = q_noised.ToQuaternion();
        localization_ptr->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qx(orientation_noised.x());
        localization_ptr->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qy(orientation_noised.y());
        localization_ptr->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qz(orientation_noised.z());
        localization_ptr->mutable_loc_pose()->mutable_pose()->mutable_orientation()->set_qw(orientation_noised.w());
    }

    return true;
}

bool NoiseGenerator::UpdateLocalizationInfo(const pnc::localization::LocalizationEstimate* const localization_ptr) {
    if (localization_ptr == nullptr) {
        LOG_WARN("NOISE:localization_ptr is nullptr");
        return false;
    }

    double x_origin = localization_ptr->loc_pose().pose().position().x();
    double y_origin = localization_ptr->loc_pose().pose().position().y();
    localzation_info_.pure_x = x_origin;
    localzation_info_.pure_y = y_origin;

    auto orientation = localization_ptr->loc_pose().pose().orientation();
    jiduauto::pnc::EulerAnglesZXYf q_origin(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
    double yaw_origin = q_origin.yaw();
    double roll_origin = q_origin.roll();
    double pitch_origin = q_origin.pitch();
    localzation_info_.pure_yaw = yaw_origin;
    localzation_info_.pure_roll = roll_origin;
    localzation_info_.pure_pitch = pitch_origin;

    return true;
}

void NoiseGenerator::GenerateNoisedXY(LocalizationInformation* const localzation_info_ptr) {
    const double kDisturbPosition = noise_generator_conf_.noise_localization_xy();
    const double kDisturbX = kDisturbPosition / std::sqrt(2);
    const double kDisturbY = kDisturbPosition / std::sqrt(2);
    // using 3-sigma principle
    const double kSigmaX = kDisturbX / 3;
    const double kSigmaY = kDisturbY / 3;
    double error_x = GenerateGaussianNoise(0, kSigmaX);
    double error_y = GenerateGaussianNoise(0, kSigmaY);

    double x_origin = localzation_info_ptr->pure_x;
    double y_origin = localzation_info_ptr->pure_y;
    double x_noised = x_origin + error_x;
    double y_noised = y_origin + error_y;
    localzation_info_ptr->noised_x = x_noised;
    localzation_info_ptr->noised_y = y_noised;
    LOG_INFO("Noise:localization original x:%.4f, localization original y:%.4f", x_origin, y_origin);
    LOG_INFO("Noise:error_x:%.4f,error_y:%.4f", error_x, error_y);
    LOG_INFO("Noise:localization noised x:%.4f, Noise:localization noised y:%.4f", x_noised, y_noised);
}

void NoiseGenerator::GenerateNoisedYaw(LocalizationInformation* const localzation_info_ptr) {
    const double kDisturbYawDegree = noise_generator_conf_.noise_localization_yaw();
    const double kDisturbYawRad = kDisturbYawDegree * M_PI / 180.0;
    const double kSigmaYawRad = kDisturbYawRad / 3;
    double error_heading = GenerateGaussianNoise(0, kSigmaYawRad);

    double yaw_origin = localzation_info_ptr->pure_yaw;
    double yaw_noised = yaw_origin + error_heading;
    localzation_info_ptr->noised_yaw = yaw_noised;
    LOG_INFO("Noise:localization original yaw:%.4f", yaw_origin);
    LOG_INFO("Control Message:error_heading:%.4f", error_heading);
    LOG_INFO("Noise:localization noised heading:%.4f", yaw_noised);
}

void NoiseGenerator::SetDebugInfo(pnc::control::ControlDebug* const control_debug) {
    pnc::control::NoiseGeneratorDebug* noise_generator_debug = control_debug->mutable_noise_generator_debug();
    noise_generator_debug->set_pure_x(localzation_info_.pure_x);
    noise_generator_debug->set_pure_y(localzation_info_.pure_y);
    noise_generator_debug->set_pure_yaw(localzation_info_.pure_yaw);
}

double NoiseGenerator::GenerateGaussianNoise(const double mean, const double sigma) {
    std::uniform_real_distribution<double> dist1(0.0, 1.0);
    std::uniform_real_distribution<double> dist2(0.0, 1.0);
    std::random_device rd_;
    std::mt19937 rng(rd_());
    double tmp_a{0.0}, tmp_b{0.0}, tmp_r{0.0};
    double gaussian_noise{0.0};
    while (tmp_r > 1.0 || tmp_r == 0.0) {
        tmp_a = -1.0 + 2.0 * dist1(rng);
        tmp_b = -1.0 + 2.0 * dist2(rng);
        tmp_r = tmp_a * tmp_a + tmp_b * tmp_b;
    }
    gaussian_noise = mean + sigma * tmp_a * std::sqrt(-2.0 * log(tmp_r) / tmp_r);
    return gaussian_noise;
}

}  // namespace control
}  // namespace jiduauto
