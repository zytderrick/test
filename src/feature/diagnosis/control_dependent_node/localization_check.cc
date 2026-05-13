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
#include "control/src/feature/diagnosis/control_dependent_node/localization_check.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "control/src/common/control_gflags.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/transform/euler_angles_zxy.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::util::TimeUtility;

bool LocalizationCheck::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("Diagnosis: LocalizationCheck Init, starting ...");

    if (!SanityCheck(control_conf)) {
        LOG_ERROR("Diagnosis: SanityCheck failed");
        return false;
    }

    veh_paras_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();

    control_conf_ = control_conf;

    return true;
}

void LocalizationCheck::Reset() {
    global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    global_paras_.check_result_string_vector.clear();
    global_paras_.localization_check_code = 0;
    SetCodeBitHigh(&global_paras_.localization_check_code,
                   pnc::control::LocalizationCheckBit::LOCALIZATION_UNKNOWN_BIT);
    ClearQueue(&global_paras_.localization_check_code_queue);

    global_paras_.general_check_paras.count_timestamp_check = 0;
    global_paras_.general_check_paras.last_timestamp = TimeUtility::GetCurrentTimesecond();

    global_paras_.specific_check_paras.localization_not_jump_paras.pre_excute_localization_jump_check = false;
    global_paras_.specific_check_paras.localization_not_jump_paras.excute_localization_jump_check = false;
    global_paras_.specific_check_paras.localization_not_jump_paras.localization_jump_count = 0;
    global_paras_.specific_check_paras.localization_not_jump_paras.pre_x_pos = 0.0;
    global_paras_.specific_check_paras.localization_not_jump_paras.pre_y_pos = 0.0;
    global_paras_.specific_check_paras.localization_not_jump_paras.pre_heading = 0.0;
    global_paras_.specific_check_paras.localization_not_jump_paras.pre_speed = 0.0;
    global_paras_.specific_check_paras.localization_not_jump_paras.pre_acc = 0.0;
    global_paras_.specific_check_paras.localization_not_jump_paras.pre_wheel_steer_angle_rad = 0.0;
    global_paras_.specific_check_paras.localization_not_jump_paras.localization_jump_return_code =
        LocalizationCheck::LocalizationJumpReturnCode::NORMAL;
    ClearQueue(&global_paras_.specific_check_paras.localization_not_jump_paras.distance_error_queue);
}

std::string LocalizationCheck::Name() const { return "Localization"; }

bool LocalizationCheck::SanityCheck(const pnc::control::ControlConfig& control_conf) {
    // Note: Please synchronize the new parameters to ./path_to_unit_test/data/control_conf.pb.txt

    if (!control_conf.has_diag_v2_conf()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: diag_v2_conf");
        return false;
    }

    const pnc::control::DiagnosisV2Conf diag_v2_conf = control_conf.diag_v2_conf();
    if (!diag_v2_conf.has_frames_jump_timestamp_check() || !diag_v2_conf.has_enable_localization_jump_check() ||
        !diag_v2_conf.has_localization_check_conf()) {
        LOG_ERROR(
            "Diagnosis: Fail to check conf paras: frames_jump_timestamp_check or enable_localization_jump_check or "
            "localization_check_conf");
        return false;
    }

    const pnc::control::LocalizationCheckConf localization_check_conf = diag_v2_conf.localization_check_conf();
    if (!localization_check_conf.has_trajectory_localization_distance_threshold() ||
        !localization_check_conf.has_localization_permit_distance_error_first_threshold() ||
        !localization_check_conf.has_localization_permit_distance_error_second_threshold() ||
        !localization_check_conf.has_localization_permit_heading_error_threshold()) {
        LOG_ERROR(
            "Diagnosis: Fail to check conf paras: trajectory_localization_distance_threshold or "
            "localization_permit_distance_error_threshold or localization_permit_heading_error_threshold");
        return false;
    }

    return true;
}

bool LocalizationCheck::GeneralCheck(const InputStream& input_stream,
                                     pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    global_paras_.localization_check_code = 0;
    SetCodeBitHigh(&global_paras_.localization_check_code,
                   pnc::control::LocalizationCheckBit::LOCALIZATION_UNKNOWN_BIT);

    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: Fail to check diag_debug");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        return false;
    }

    // clear check_result_string_vector
    global_paras_.check_result_string_vector.clear();

    // check pointer
    const pnc::localization::LocalizationEstimate* const localization_ptr = &input_stream.localization;
    if (localization_ptr == nullptr) {
        global_paras_.check_result_string_vector.push_back("nullptr");
        SetCodeBitHigh(&global_paras_.localization_check_code,
                       pnc::control::LocalizationCheckBit::LOCALIZATION_NULLPTR_BIT);
        LOG_DEBUG("Diagnosis: localization_ptr is nullptr");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    // check header
    if (localization_ptr != nullptr && !localization_ptr->has_header()) {
        global_paras_.check_result_string_vector.push_back("header empty");
        SetCodeBitHigh(&global_paras_.localization_check_code,
                       pnc::control::LocalizationCheckBit::LOCALIZATION_HEADER_BIT);
        LOG_DEBUG("Diagnosis: localization header is empty");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    if (localization_ptr != nullptr && localization_ptr->has_header()) {
        // check timestamp
        if (std::isnan(localization_ptr->header().timestamp_sec()) ||
            TimestampCheck(localization_ptr->header().timestamp_sec(), &global_paras_.general_check_paras) ==
                pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            global_paras_.check_result_string_vector.push_back("timestamp abnormal");
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_TIMESTAMP_BIT);
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_TIMESTAMP_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: localization timestamp_sec is abnormal");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check status (Notice ANP may not have status in header)
        if (localization_ptr->header().has_status() &&
            localization_ptr->header().status().error_code() != pnc::common::ErrorCode::OK) {
            global_paras_.check_result_string_vector.push_back("bad status");
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_STATUS_BIT);
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_STATUS_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: localization status is not OK");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check pose
        if (!localization_ptr->has_loc_pose() || !localization_ptr->loc_pose().has_local_pose() ||
            !localization_ptr->loc_pose().local_pose().has_position() ||
            !localization_ptr->loc_pose().local_pose().has_orientation() ||
            !localization_ptr->loc_pose().local_pose().has_linear_velocity() ||
            !localization_ptr->loc_pose().local_pose().has_linear_acceleration()) {
            global_paras_.check_result_string_vector.push_back("pose abnormal");
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_POSE_BIT);
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_POSE_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: localization pose is abnormal");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }
    }

    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP) {
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
    }

    bool bflag{true};
    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE) {
        bflag = true;
    } else {
        bflag = false;
        SetCodeBitHigh(&global_paras_.localization_check_code,
                       pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT);
        SetCodeBitHigh(&global_paras_.localization_check_code,
                       pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_LEVEL_BIT);
    }
    SetCodeBitLow(&global_paras_.localization_check_code, pnc::control::LocalizationCheckBit::LOCALIZATION_UNKNOWN_BIT);

    SetDebugInfo(global_paras_, diag_debug_ptr);

    return bflag;
}

pnc::control::DiagnosisReturnCode LocalizationCheck::TimestampCheck(const double timestamp_sec,
                                                                    GeneralCheckParas* const general_check_paras_ptr) {
    double time_interval_control2message =
        TimeUtility::GetCurrentTimesecond() - general_check_paras_ptr->last_timestamp;
    double time_interval_message2last = timestamp_sec - general_check_paras_ptr->last_timestamp;
    general_check_paras_ptr->last_timestamp = TimeUtility::GetCurrentTimesecond();
    if (std::max(std::fabs(time_interval_message2last), time_interval_control2message) >
        control_conf_.diag_v2_conf().localization_check_conf().max_loc_interval_sec()) {
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

bool LocalizationCheck::SpecificCheck(const ControlInputStream& control_input_stream,
                                      pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: Fail to check diag_debug");
        return false;
    }

    bool bflag{true};

    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP) {
        global_paras_.check_result_string_vector.push_back("diagnosis skip");
        SetCodeBitHigh(&global_paras_.localization_check_code,
                       pnc::control::LocalizationCheckBit::LOCALIZATION_UNKNOWN_BIT);
        bflag = false;
    } else if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
        // general check failed, specific check return false
        SetCodeBitHigh(&global_paras_.localization_check_code,
                       pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT);
        SetCodeBitHigh(&global_paras_.localization_check_code,
                       pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_LEVEL_BIT);
        SetCodeBitLow(&global_paras_.localization_check_code,
                      pnc::control::LocalizationCheckBit::LOCALIZATION_UNKNOWN_BIT);
        if (global_paras_.check_result_string_vector.empty()) {
            global_paras_.check_result_string_vector.push_back("fail to show error");
        }
        bflag = false;
    } else {
        const pnc::localization::LocalizationEstimate* const localization_ptr =
            &control_input_stream.input_stream.localization;
        const pnc::planning::ADCTrajectory* const trajectory_ptr = &control_input_stream.input_stream.planning_info;

        // check function

        if (CheckDistanceFromTrajectoryFirstPoint(localization_ptr, trajectory_ptr) ==
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            global_paras_.check_result_string_vector.push_back("bad distance");
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_DISTANCE_BIT);
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_DISTANCE_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: Bad DistanceFromTrajectoryFirstPoint");
            bflag = false;
        }

        if (!LocalizationNotJump(control_input_stream,
                                 &global_paras_.specific_check_paras.localization_not_jump_paras)) {
            // Note: The current strategy of localization jump check divide the result into small jump and big jump
            // small jump:jump but low level, no response immediately
            // big jump:jump and high level， response immediately
            if (global_paras_.specific_check_paras.localization_not_jump_paras.localization_jump_return_code ==
                LocalizationCheck::LocalizationJumpReturnCode::FIRST_LEVEL_JUMP) {
                global_paras_.check_result_string_vector.push_back("small jump");
                SetCodeBitHigh(&global_paras_.localization_check_code,
                               pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_BIT);
                SetCodeBitLow(&global_paras_.localization_check_code,
                              pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_LEVEL_BIT);
                if (FLAGS_enable_control_info_print) {
                    LOG_INFO("Diagnosis: Localization Small Jump");
                }
            } else if (global_paras_.specific_check_paras.localization_not_jump_paras.localization_jump_return_code ==
                       LocalizationCheck::LocalizationJumpReturnCode::SECOND_LEVEL_JUMP) {
                global_paras_.check_result_string_vector.push_back("big jump");
                SetCodeBitHigh(&global_paras_.localization_check_code,
                               pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_BIT);
                SetCodeBitHigh(&global_paras_.localization_check_code,
                               pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_LEVEL_BIT);
                LOG_DEBUG("Diagnosis: Localization Big Jump");
            } else if (global_paras_.specific_check_paras.localization_not_jump_paras.localization_jump_return_code ==
                       LocalizationCheck::LocalizationJumpReturnCode::ACCUMULATED_ERROR) {
                global_paras_.check_result_string_vector.push_back("accumulated error");
                SetCodeBitHigh(&global_paras_.localization_check_code,
                               pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_BIT);
                SetCodeBitHigh(&global_paras_.localization_check_code,
                               pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_LEVEL_BIT);

                LOG_DEBUG("Diagnosis: Localization Accumulated Error");
            }
            // check if localization jump level bit is high
            bool is_jump_level_bit_high =
                ((global_paras_.localization_check_code &
                  (0x00000001 << (pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_LEVEL_BIT - 1))) >>
                 (pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_LEVEL_BIT - 1)) == 0x00000001;
            if (control_conf_.diag_v2_conf().enable_localization_jump_check() && is_jump_level_bit_high) {
                bflag = false;
            }
        }

        // continuous check
        std::vector<std::string> continuous_check_result_string_vector;
        if (!LocalizationContinuousCheck(global_paras_.localization_check_code_queue,
                                         &continuous_check_result_string_vector)) {
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_CONTINUOUS_ABNORMAL_BIT);
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_CONTINUOUS_ABNORMAL_LEVEL_BIT);
            if (!continuous_check_result_string_vector.empty()) {
                for (int i = 0; i < static_cast<int>(continuous_check_result_string_vector.size()); ++i) {
                    global_paras_.check_result_string_vector.push_back(continuous_check_result_string_vector[i]);
                }
            }
            if (control_conf_.diag_v2_conf().enable_localization_continuous_check()) {
                bflag = false;
            }
        }

        // postprocess,set string
        if (!global_paras_.check_result_string_vector.empty()) {
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_BIT);
            SetCodeBitHigh(&global_paras_.localization_check_code,
                           pnc::control::LocalizationCheckBit::LOCALIZATION_SUMMARY_LEVEL_BIT);
            SetCodeBitLow(&global_paras_.localization_check_code,
                          pnc::control::LocalizationCheckBit::LOCALIZATION_UNKNOWN_BIT);
        }
    }

    UpdateCodeQueue(&global_paras_);

    SetDebugInfo(global_paras_, diag_debug_ptr);

    return bflag;
}

// private
pnc::control::DiagnosisReturnCode LocalizationCheck::CheckDistanceFromTrajectoryFirstPoint(
    const pnc::localization::LocalizationEstimate* const localization_ptr,
    const pnc::planning::ADCTrajectory* const trajectory_ptr) {
    pnc::control::DiagnosisReturnCode code_skip = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    // check input
    if (localization_ptr == nullptr || trajectory_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: localization_ptr or trajectory_ptr is nullptr");
        return code_skip;
    }

    // Note: always get loc pose (no matter FLAGS_enable_using_global_localization)
    if (!localization_ptr->has_loc_pose() || !localization_ptr->loc_pose().has_pose()) {
        LOG_DEBUG("Diagnosis: loc_pose or pose is missing");
        return code_skip;
    }
    pnc::localization::Pose localization_pose = localization_ptr->loc_pose().pose();

    if (trajectory_ptr->trajectory_point_size() == 0 || !trajectory_ptr->trajectory_point()[0].has_path_point()) {
        LOG_DEBUG("Diagnosis: trajectory has no point size or path_point of the first point is missing");
        return code_skip;
    }
    pnc::common::PathPoint trajectory_first_pose = trajectory_ptr->trajectory_point()[0].path_point();

    double distance_trajectory_localization =
        std::sqrt((localization_pose.position().x() - trajectory_first_pose.x()) *
                      (localization_pose.position().x() - trajectory_first_pose.x()) +
                  (localization_pose.position().y() - trajectory_first_pose.y()) *
                      (localization_pose.position().y() - trajectory_first_pose.y()));

    if (!control_conf_.has_diag_v2_conf() || !control_conf_.diag_v2_conf().has_localization_check_conf() ||
        !control_conf_.diag_v2_conf().localization_check_conf().has_trajectory_localization_distance_threshold()) {
        LOG_ERROR(
            "Diagnosis: diag_v2_conf or localization_check_conf or trajectory_localization_distance_threshold in "
            "control_conf is missing");
        return code_skip;
    }
    if (distance_trajectory_localization >
        control_conf_.diag_v2_conf().localization_check_conf().trajectory_localization_distance_threshold()) {
        LOG_DEBUG("Diagnosis: distance_trajectory_localization is %f, greater than threshold: %f",
                  distance_trajectory_localization,
                  control_conf_.diag_v2_conf().localization_check_conf().trajectory_localization_distance_threshold());
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

bool LocalizationCheck::LocalizationNotJump(const ControlInputStream& control_input_stream,
                                            LocalizationNotJumpParas* const localization_not_jump_paras_ptr) {
    // TODO(@liang.zhu): Move LocalizationJumped in diagnosis_v1 here
    // Note: To maintain consistency of the function format，if the function is abnormal, return false
    const pnc::localization::LocalizationEstimate* const localization_ptr =
        &control_input_stream.input_stream.localization;
    const pnc::chassis::Chassis* const chassis_ptr = &control_input_stream.input_stream.chassis;
    if (localization_ptr == nullptr || chassis_ptr == nullptr) {
        localization_not_jump_paras_ptr->excute_localization_jump_check = false;
        localization_not_jump_paras_ptr->localization_jump_return_code =
            LocalizationCheck::LocalizationJumpReturnCode::SKIP;
        return true;
    }

    // Reference Code For @liang.zhu
    // 上一帧不检测，当前帧也不检测
    if (!localization_not_jump_paras_ptr->pre_excute_localization_jump_check) {
        localization_not_jump_paras_ptr->excute_localization_jump_check = false;
    }
    localization_not_jump_paras_ptr->localization_jump_return_code =
        LocalizationCheck::LocalizationJumpReturnCode::NORMAL;

    bool xy_jump_flag{false};
    bool heading_jump_flag{false};
    bool bflag{true};
    double pre_x_pos{0.0}, pre_y_pos{0.0}, pre_heading{0.0}, pre_speed{0.0}, pre_acc{0.0},
        pre_wheel_steer_angle_rad{0.0};
    GetPreState(localization_not_jump_paras_ptr, &pre_x_pos, &pre_y_pos, &pre_heading, &pre_speed, &pre_acc,
                &pre_wheel_steer_angle_rad);

    double cur_x{0.0}, cur_y{0.0}, cur_heading{0.0}, cur_speed{0.0}, cur_acc{0.0}, cur_wheel_steer_angle_rad{0.0};
    if (!GetCurrentState(control_input_stream, &cur_x, &cur_y, &cur_heading, &cur_speed, &cur_acc,
                         &cur_wheel_steer_angle_rad)) {
        localization_not_jump_paras_ptr->excute_localization_jump_check = false;
    }
    LOG_DEBUG("Diagnosis:cur_x(%.4f),cur_y(%.4f),pre_x_pos(%.4f),pre_y_pos(%.4f)", cur_x, cur_y, pre_x_pos, pre_y_pos);

    double expect_x_pos{0.0}, expect_y_pos{0.0}, expect_heading{0.0};
    GetPredictState(pre_x_pos, pre_y_pos, pre_heading, pre_speed, pre_acc, pre_wheel_steer_angle_rad, &expect_x_pos,
                    &expect_y_pos, &expect_heading);

    double station_error{0.0}, lateral_error{0.0}, heading_error{0.0}, distance_error{0.0};
    CalculateLocalizationError(cur_x, cur_y, cur_heading, expect_x_pos, expect_y_pos, expect_heading, &station_error,
                               &lateral_error, &heading_error, &distance_error);

    UpdatePrestate(localization_not_jump_paras_ptr, cur_x, cur_y, cur_heading, cur_speed, cur_acc,
                   cur_wheel_steer_angle_rad);

    double ts = control_conf_.control_period();
    pnc::control::LocalizationCheckConf localization_check_conf =
        control_conf_.diag_v2_conf().localization_check_conf();
    const double kFirstLevelSpeedMultiplier = localization_check_conf.localization_jump_first_level_speed_multiplier();
    const double kSecondLevelSpeedMultiplier =
        localization_check_conf.localization_jump_second_level_speed_multiplier();
    double first_level_permit_distance_error = kFirstLevelSpeedMultiplier * pre_speed * ts;
    double second_level_permit_distance_error = kSecondLevelSpeedMultiplier * pre_speed * ts;
    const double kPermitDistanceErrorFirstThreshold =
        localization_check_conf.localization_permit_distance_error_first_threshold();
    const double kPermitDistanceErrorSecondThreshold =
        localization_check_conf.localization_permit_distance_error_second_threshold();
    const double kPermitStationErrorThreshold = localization_check_conf.localization_jump_station_error_threshold();
    const double kPermitLateralErrorThreshold = localization_check_conf.localization_jump_lateral_error_threshold();
    first_level_permit_distance_error = std::max(first_level_permit_distance_error, kPermitDistanceErrorFirstThreshold);
    second_level_permit_distance_error =
        std::max(second_level_permit_distance_error, kPermitDistanceErrorSecondThreshold);

    const double kCheckZeroThreshold = 1e-5;
    if (std::fabs(pre_x_pos - cur_x) < kCheckZeroThreshold && std::fabs(pre_y_pos - cur_y) < kCheckZeroThreshold) {
        LOG_DEBUG("Diagnosis:localization xy position did not change !!!");
    }
    if (std::fabs(pre_heading - cur_heading) < kCheckZeroThreshold) {
        LOG_DEBUG("Diagnosis:localization heading did not change !!!");
    }
    if (std::fabs(pre_x_pos) < kCheckZeroThreshold && std::fabs(pre_y_pos) < kCheckZeroThreshold) {
        LOG_DEBUG("Diagnosis:previous position is (0.0,0.0), not considered as jump");
        localization_not_jump_paras_ptr->excute_localization_jump_check = false;
    }

    if ((localization_not_jump_paras_ptr->excute_localization_jump_check) &&
        (std::fabs(station_error) > kPermitStationErrorThreshold ||
         std::fabs(lateral_error) > kPermitLateralErrorThreshold ||
         distance_error > first_level_permit_distance_error)) {
        LOG_WARN("Diagnosis:Localization information: xy does jump!");
        xy_jump_flag = true;
    }
    const double kPermitHeadingErrorThreshold = localization_check_conf.localization_permit_heading_error_threshold();
    if (localization_not_jump_paras_ptr->excute_localization_jump_check &&
        heading_error > kPermitHeadingErrorThreshold) {
        LOG_DEBUG("Diagnosis:cur_heading:%.4f,expect_heading:%.4f,pre_heading:%.4f", cur_heading, expect_heading,
                  pre_heading);
        LOG_WARN("Diagnosis:Localization information: heading does jump!");
        heading_jump_flag = true;
    }

    double accumulated_distance_error{0.0};
    if (localization_not_jump_paras_ptr->excute_localization_jump_check) {
        localization_not_jump_paras_ptr->distance_error_queue.push(distance_error);
        while (static_cast<int>(localization_not_jump_paras_ptr->distance_error_queue.size()) >
               localization_check_conf.localization_jump_accumulated_error_duration()) {
            localization_not_jump_paras_ptr->distance_error_queue.pop();
        }
        accumulated_distance_error =
            LocalizationJumpAccumulatedError(localization_not_jump_paras_ptr->distance_error_queue);
    }

    if ((xy_jump_flag || heading_jump_flag) && localization_not_jump_paras_ptr->localization_jump_count < INT_MAX) {
        localization_not_jump_paras_ptr->localization_jump_count += 1;
    }

    // 连续两帧没检测，第三帧不应该默认不检测
    if (!localization_not_jump_paras_ptr->excute_localization_jump_check &&
        !localization_not_jump_paras_ptr->pre_excute_localization_jump_check) {
        localization_not_jump_paras_ptr->pre_excute_localization_jump_check = true;
    } else {
        localization_not_jump_paras_ptr->pre_excute_localization_jump_check =
            localization_not_jump_paras_ptr->excute_localization_jump_check;
    }
    localization_not_jump_paras_ptr->excute_localization_jump_check = true;

    localization_not_jump_paras_ptr->localization_xy_jump = xy_jump_flag;
    localization_not_jump_paras_ptr->localization_heading_jump = heading_jump_flag;
    localization_not_jump_paras_ptr->accumulated_distance_error = accumulated_distance_error;

    if (xy_jump_flag || heading_jump_flag) {
        // judge the jump level
        localization_not_jump_paras_ptr->localization_jump_return_code =
            LocalizationCheck::LocalizationJumpReturnCode::FIRST_LEVEL_JUMP;
        bflag = false;
    }
    if (xy_jump_flag && distance_error > second_level_permit_distance_error) {
        localization_not_jump_paras_ptr->localization_jump_return_code =
            LocalizationCheck::LocalizationJumpReturnCode::SECOND_LEVEL_JUMP;
        bflag = false;
    }
    if (accumulated_distance_error > localization_check_conf.localization_jump_accumulated_distance_error_threshold()) {
        localization_not_jump_paras_ptr->localization_jump_return_code =
            LocalizationCheck::LocalizationJumpReturnCode::ACCUMULATED_ERROR;
        bflag = false;
    }
    return bflag;
}

bool LocalizationCheck::GetCurrentState(const ControlInputStream& control_input_stream, double* const x,
                                        double* const y, double* const heading, double* const speed, double* const acc,
                                        double* const wheel_steer_angle_rad) {
    bool bflag{true};

    const pnc::chassis::Chassis* const chassis_ptr = &control_input_stream.input_stream.chassis;
    const pnc::localization::LocalizationEstimate* const localization_ptr =
        &control_input_stream.input_stream.localization;
    pnc::localization::Pose localization_pose = localization_ptr->loc_pose().pose();
    *x = localization_pose.position().x();
    *y = localization_pose.position().y();
    *heading = control_input_stream.vehicle_state.heading();
    if (fabs(*heading) > M_PI) {
        LOG_WARN("Diagnosis:abs(heading) > M_PI!!");
        bflag = false;
    }
    if (chassis_ptr->has_speed_mps() && !std::isnan(chassis_ptr->speed_mps())) {
        *speed = chassis_ptr->speed_mps();
    } else {
        bflag = false;
    }
    bool curr_backward = ControlMath::CheckBackward(chassis_ptr);
    if (curr_backward) {
        (*speed) *= -1.0;
    }

    if ((chassis_ptr->has_chassis_lon_acc() && !std::isnan(chassis_ptr->chassis_lon_acc()))) {
        *acc = chassis_ptr->chassis_lon_acc();
    } else {
        bflag = false;
    }

    if (chassis_ptr->has_steering_percentage() && !std::isnan(chassis_ptr->steering_percentage())) {
        *wheel_steer_angle_rad =
            chassis_ptr->steering_percentage() / 100.0 * veh_paras_.max_steer_angle() / veh_paras_.steer_ratio();
    } else {
        bflag = false;
    }

    return bflag;
}

void LocalizationCheck::GetPreState(const LocalizationNotJumpParas* const localization_not_jump_paras_ptr,
                                    double* const pre_x, double* const pre_y, double* const pre_heading,
                                    double* const pre_speed, double* const pre_acc,
                                    double* const pre_wheel_steer_angle_rad) {
    *pre_x = localization_not_jump_paras_ptr->pre_x_pos;
    *pre_y = localization_not_jump_paras_ptr->pre_y_pos;
    *pre_heading = localization_not_jump_paras_ptr->pre_heading;
    *pre_speed = localization_not_jump_paras_ptr->pre_speed;
    *pre_acc = localization_not_jump_paras_ptr->pre_acc;
    *pre_wheel_steer_angle_rad = localization_not_jump_paras_ptr->pre_wheel_steer_angle_rad;
}

void LocalizationCheck::GetPredictState(const double& pre_x, const double& pre_y, const double& pre_heading,
                                        const double& pre_speed, const double& pre_acc,
                                        const double& pre_wheel_steer_angle_rad, double* const expect_x,
                                        double* const expect_y, double* const expect_heading) {
    double ts = control_conf_.control_period();
    double speed_average = pre_speed + pre_acc * ts * 0.5;
    // *expect_x =
    //     pre_x - pre_speed * std::sin(pre_heading) * ts * pre_heading + pre_speed * std::cos(pre_heading) * ts;
    // *expect_y =
    //     pre_y + pre_speed * std::cos(pre_heading) * ts * pre_heading + pre_speed * std::sin(pre_heading) * ts;
    *expect_x = pre_x + std::cos(pre_heading) * speed_average * ts;
    *expect_y = pre_y + std::sin(pre_heading) * speed_average * ts;
    *expect_heading = pre_heading + std::tan(pre_wheel_steer_angle_rad) * speed_average * ts / veh_paras_.wheel_base();
    *expect_heading = jiduauto::pnc::math::NormalizeAngle(*expect_heading);
    LOG_DEBUG(
        "Diagnosis:expect_x_pos(%.4f) = pre_x_pos(%.4f) + cos(pre_heading(%.4f)) * speed_average(%.4f) * "
        "ts(%.4f),cos(pre_heading)=(%.4f)",
        *expect_x, pre_x, pre_heading, speed_average, ts, std::cos(pre_heading));
    LOG_DEBUG(
        "Diagnosis:expect_y_pos(%.4f) = pre_y_pos(%.4f) + sin(pre_heading(%.4f)) * speed_average(%.4f) * "
        "ts(%.4f),sin(pre_heading)=(%.4f)",
        *expect_y, pre_y, pre_heading, speed_average, ts, std::sin(pre_heading));
}

void LocalizationCheck::UpdatePrestate(LocalizationNotJumpParas* const localization_not_jump_paras_ptr, const double& x,
                                       const double& y, const double& heading, const double& speed, const double& acc,
                                       const double& wheel_steer_angle_rad) {
    localization_not_jump_paras_ptr->pre_x_pos = x;
    localization_not_jump_paras_ptr->pre_y_pos = y;
    localization_not_jump_paras_ptr->pre_heading = heading;
    localization_not_jump_paras_ptr->pre_speed = speed;
    localization_not_jump_paras_ptr->pre_acc = acc;
    localization_not_jump_paras_ptr->pre_wheel_steer_angle_rad = wheel_steer_angle_rad;
}

bool LocalizationCheck::LocalizationContinuousCheck(
    const std::queue<int>& localization_check_code_queue,
    std::vector<std::string>* const continuous_check_result_str_vector) {
    bool bflag{true};
    if (localization_check_code_queue.empty()) {
        return true;
    }

    // check continuous localization jump
    if (static_cast<int>(localization_check_code_queue.size()) <
        control_conf_.diag_v2_conf().localization_check_conf().localization_check_continuous_jump_threshold()) {
        bflag = true;
    } else {
        int count_localization_continuous_jump{0};
        count_localization_continuous_jump = LocalizationContinuousJumpCount(localization_check_code_queue);
        if (count_localization_continuous_jump >=
            control_conf_.diag_v2_conf().localization_check_conf().localization_check_continuous_jump_threshold()) {
            bflag = false;
            continuous_check_result_str_vector->push_back("continuous jump");
        }
    }

    return bflag;
}

int LocalizationCheck::LocalizationContinuousJumpCount(const std::queue<int>& localization_check_code_queue) {
    int count{0};
    std::queue<int> localization_check_code_queue_tmp = localization_check_code_queue;
    // delete abundant code in the queue
    while (static_cast<int>(localization_check_code_queue_tmp.size()) >
           control_conf_.diag_v2_conf().localization_check_conf().localization_check_continuous_jump_duration()) {
        localization_check_code_queue_tmp.pop();
    }
    // count jump using code left in the queue
    while (!localization_check_code_queue_tmp.empty()) {
        int localization_check_code = localization_check_code_queue_tmp.front();
        bool is_localization_jump{false};
        is_localization_jump = ((localization_check_code &
                                 (0x00000001 << (pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_BIT - 1))) >>
                                (pnc::control::LocalizationCheckBit::LOCALIZATION_JUMP_BIT - 1)) == 0x00000001;
        if (is_localization_jump) {
            ++count;
        }
        localization_check_code_queue_tmp.pop();
    }
    return count;
}

double LocalizationCheck::LocalizationJumpAccumulatedError(const std::queue<double>& error_queue) {
    double accumulated_error{0};
    std::queue<double> error_queue_tmp = error_queue;
    while (static_cast<int>(error_queue_tmp.size()) >
           control_conf_.diag_v2_conf().localization_check_conf().localization_jump_accumulated_error_duration()) {
        error_queue_tmp.pop();
    }
    while (!error_queue_tmp.empty()) {
        accumulated_error += error_queue_tmp.front();
        error_queue_tmp.pop();
    }
    return accumulated_error;
}

void LocalizationCheck::SetDebugInfo(const GlobalParas& global_param,
                                     pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    if (diag_debug_ptr == nullptr) {
        LOG_WARN("Diagnosis:SetDebugInfo failed, diag_debug_ptr is nullptr");
        return;
    }

    std::string check_string;
    if (global_param.check_result_string_vector.empty()) {
        check_string = "No error";
    } else {
        check_string = global_param.check_result_string_vector[0];
        if (static_cast<int>(global_param.check_result_string_vector.size()) > 1) {
            for (int i = 1; i < static_cast<int>(global_param.check_result_string_vector.size()); ++i) {
                check_string += " & ";
                check_string += global_param.check_result_string_vector[i];
            }
        }
    }

    diag_debug_ptr->set_localization_check_string(check_string);
    diag_debug_ptr->mutable_localization_jump_debug()->set_localization_xy_jump(
        global_param.specific_check_paras.localization_not_jump_paras.localization_xy_jump);
    diag_debug_ptr->mutable_localization_jump_debug()->set_localization_heading_jump(
        global_param.specific_check_paras.localization_not_jump_paras.localization_heading_jump);
    diag_debug_ptr->mutable_localization_jump_debug()->set_localization_jump_count(
        global_param.specific_check_paras.localization_not_jump_paras.localization_jump_count);
    diag_debug_ptr->mutable_localization_jump_debug()->set_accumulated_distanece_error(
        global_param.specific_check_paras.localization_not_jump_paras.accumulated_distance_error);
    diag_debug_ptr->set_localization_check_code(global_paras_.localization_check_code);
}

void LocalizationCheck::UpdateCodeQueue(GlobalParas* const global_param) {
    const int kCheckDuration =
        control_conf_.diag_v2_conf().localization_check_conf().localization_continuous_check_duration();
    global_param->localization_check_code_queue.push(global_param->localization_check_code);

    while (static_cast<int>(global_param->localization_check_code_queue.size()) > kCheckDuration) {
        global_param->localization_check_code_queue.pop();
    }
}

void LocalizationCheck::CalculateLocalizationError(const double& cur_x, const double& cur_y, const double& cur_heading,
                                                   const double& expect_x, const double& expect_y,
                                                   const double& expect_heading, double* const station_error,
                                                   double* const lateral_error, double* const heading_error,
                                                   double* const distance_error) {
    double dx = cur_x - expect_x;
    double dy = cur_y - expect_y;
    *distance_error = std::sqrt(std::pow(expect_x - cur_x, 2) + std::pow(expect_y - cur_y, 2));
    *heading_error = jiduauto::pnc::math::NormalizeAngle(cur_heading - expect_heading);
    *lateral_error = cos(expect_heading) * dy - sin(expect_heading) * dx;
    *station_error = -cos(expect_heading) * dx - sin(expect_heading) * dy;
}

}  // namespace control
}  // namespace jiduauto
