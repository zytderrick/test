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
#include "control/src/feature/diagnosis/control_dependent_node/trajectory_check.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "control/src/common/control_gflags.h"
#include "control/src/common/interpolation_1d.h"
#include "control/src/common/interpolation_angle.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/math/math_utility/math_utils.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::util::TimeUtility;

// public
bool TrajectoryCheck::Init(const pnc::control::ControlConfig& control_conf) {
    LOG_INFO("Diagnosis: TrajectoryCheck Init, starting ...");

    if (!SanityCheck(control_conf)) {
        LOG_ERROR("Diagnosis: SanityCheck failed");
        return false;
    }

    veh_paras_ = pnc::vehicle::VehicleConfig::Instance()->GetParas();

    control_conf_ = control_conf;

    return true;
}

void TrajectoryCheck::Reset() {
    global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    global_paras_.check_result_string_vector.clear();
    global_paras_.trajectory_check_code = 0;
    SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_UNKNOWN_BIT);
    ClearQueue(&global_paras_.trajectory_check_code_queue);

    global_paras_.general_check_paras.count_timestamp_check = 0;
    global_paras_.general_check_paras.last_timestamp = TimeUtility::GetCurrentTimesecond();

    global_paras_.specific_check_paras.count_bad_kappa = 0;
    global_paras_.specific_check_paras.count_bad_dkappa = 0;
}

std::string TrajectoryCheck::Name() const { return "Trajectory"; }

bool TrajectoryCheck::SanityCheck(const pnc::control::ControlConfig& control_conf) {
    // Note: Please synchronize the new parameters to ./path_to_unit_test/data/control_conf.pb.txt

    if (!control_conf.has_diag_v2_conf()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: diag_v2_conf");
        return false;
    }

    const pnc::control::DiagnosisV2Conf diag_v2_conf = control_conf.diag_v2_conf();
    if (!diag_v2_conf.has_frames_jump_timestamp_check() || !diag_v2_conf.has_enable_curvature_check()) {
        LOG_ERROR("Diagnosis: Fail to check conf paras: frames_jump_timestamp_check or enable_curvature_check");
        return false;
    }

    return true;
}

bool TrajectoryCheck::GeneralCheck(const InputStream& input_stream,
                                   pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    global_paras_.trajectory_check_code = 0;
    SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_UNKNOWN_BIT);

    // check output
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: Fail to check diag_debug");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        return false;
    }

    // clear check_result_string_vector
    global_paras_.check_result_string_vector.clear();

    // check pointer
    const pnc::planning::ADCTrajectory* const trajectory_ptr = &input_stream.planning_info;
    if (trajectory_ptr == nullptr) {
        global_paras_.check_result_string_vector.push_back("nullptr");
        SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_NULLPTR_BIT);
        LOG_DEBUG("Diagnosis: trajectory_ptr is nullptr");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    // check header
    if (trajectory_ptr != nullptr && !trajectory_ptr->has_header()) {
        global_paras_.check_result_string_vector.push_back("header empty");
        SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_HEADER_BIT);
        LOG_DEBUG("Diagnosis: header is empty in trajectory");
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    if (trajectory_ptr != nullptr && trajectory_ptr->has_header()) {
        // check timestamp
        if (std::isnan(trajectory_ptr->header().timestamp_sec()) ||
            TimestampCheck(trajectory_ptr->header().timestamp_sec(), &global_paras_.general_check_paras) ==
                pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            global_paras_.check_result_string_vector.push_back("timestamp abnormal");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_TIMESTAMP_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_TIMESTAMP_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: timestamp_sec is abnormal in trajectory");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check gear
        if (!trajectory_ptr->has_gear()) {
            global_paras_.check_result_string_vector.push_back("no gear");
            SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_GEAR_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_GEAR_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: gear_location is empty in trajectory");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        } else {
            switch (trajectory_ptr->gear()) {
            // Note: trajectory does not include case pnc::chassis::GEAR_NEUTRAL:
            case pnc::chassis::GEAR_DRIVE:
            case pnc::chassis::GEAR_REVERSE:
            case pnc::chassis::GEAR_PARKING:
                // normal, do nothing
                break;
            default:
                // abnormal
                global_paras_.check_result_string_vector.push_back("gear abnormal");
                SetCodeBitHigh(&global_paras_.trajectory_check_code,
                               pnc::control::TrajectoryCheckBit::TRAJECTORY_GEAR_BIT);
                SetCodeBitHigh(&global_paras_.trajectory_check_code,
                               pnc::control::TrajectoryCheckBit::TRAJECTORY_GEAR_LEVEL_BIT);
                LOG_DEBUG("Diagnosis: gear_location is abnormal in trajectory");
                global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
            }
        }

        // check point size
        if (trajectory_ptr->trajectory_point_size() == 0) {
            global_paras_.check_result_string_vector.push_back("no points");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_POINT_SIZE_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_POINT_SIZE_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: Trajectory has no points");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check estop
        if (EstopCheck(trajectory_ptr) == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            global_paras_.check_result_string_vector.push_back("estop");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_ESTOP_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_ESTOP_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: estop");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }

        // check relative time
        if (RelativeTimeCheck(trajectory_ptr) == pnc::control::DIAGNOSIS_RETURN_FALSE) {
            global_paras_.check_result_string_vector.push_back("bad relative time");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_RELATIVE_TIME_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_RELATIVE_TIME_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: Fail to check RelativeTimeCheck");
            global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }
    }

    // post process
    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP) {
        global_paras_.is_general_check_passed = pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
    }

    bool bflag{true};
    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE) {
        bflag = true;
    } else {
        bflag = false;
        SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_BIT);
        SetCodeBitHigh(&global_paras_.trajectory_check_code,
                       pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_LEVEL_BIT);
    }
    SetCodeBitLow(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_UNKNOWN_BIT);

    SetDebugInfo(global_paras_, diag_debug_ptr);

    return bflag;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::TimestampCheck(const double timestamp_sec,
                                                                  GeneralCheckParas* const general_check_paras_ptr) {
    double time_interval_control2message =
        TimeUtility::GetCurrentTimesecond() - general_check_paras_ptr->last_timestamp;
    double time_interval_message2last = timestamp_sec - general_check_paras_ptr->last_timestamp;
    general_check_paras_ptr->last_timestamp = TimeUtility::GetCurrentTimesecond();
    if (std::max(std::fabs(time_interval_message2last), time_interval_control2message) >
        control_conf_.diag_v2_conf().trajectory_check_conf().max_planning_interval_sec()) {
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::EstopCheck(
    const pnc::planning::ADCTrajectory* const trajectory_ptr) {
    // check input
    if (trajectory_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: trajectory_ptr is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    // Note: estop does not exist in every frame
    if (trajectory_ptr->has_estop() && trajectory_ptr->estop().has_is_estop() && trajectory_ptr->estop().is_estop()) {
        LOG_DEBUG("Diagnosis: estop is true");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }

    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::RelativeTimeCheck(
    const pnc::planning::ADCTrajectory* const trajectory_ptr) {
    // check input
    if (trajectory_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: trajectory_ptr is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    // check dependent
    int kPointSizeLimt = 2;
    if (trajectory_ptr->trajectory_point_size() < kPointSizeLimt) {
        LOG_DEBUG("Diagnosis: trajectory_point_size less than %d", kPointSizeLimt);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    // do check
    int kIndex = 0;
    if (!trajectory_ptr->trajectory_point()[kIndex].has_relative_time()) {
        LOG_DEBUG("Diagnosis: trajectory_point %d has no relative_time", kIndex);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    double last_relative_time = trajectory_ptr->trajectory_point()[kIndex].relative_time();
    const double kEpsilon = 1.0e-6;
    for (int i = 1; i < trajectory_ptr->trajectory_point_size(); ++i) {
        if (!trajectory_ptr->trajectory_point()[i].has_relative_time()) {
            LOG_DEBUG("Diagnosis: trajectory_point %d has no relative_time", i);
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        }
        double time_on_trajectory = trajectory_ptr->trajectory_point()[i].relative_time();
        double time_diff = time_on_trajectory - last_relative_time;
        if (time_diff <= kEpsilon) {
            LOG_DEBUG("Diagnosis: Bad relative time in trajectory");
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }
        last_relative_time = time_on_trajectory;
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

bool TrajectoryCheck::SpecificCheck(const ControlInputStream& control_input_stream,
                                    pnc::control::DiagnosisDebugV2* const diag_debug_ptr) {
    // checkout
    if (diag_debug_ptr == nullptr) {
        LOG_ERROR("Diagnosis: Fail to check diag_debug");
        return false;
    }

    bool bflag{true};

    if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP) {
        global_paras_.check_result_string_vector.push_back("diagnosis skip");
        SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_UNKNOWN_BIT);
        bflag = false;
    } else if (global_paras_.is_general_check_passed == pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
        SetCodeBitHigh(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_BIT);
        SetCodeBitHigh(&global_paras_.trajectory_check_code,
                       pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_LEVEL_BIT);
        SetCodeBitLow(&global_paras_.trajectory_check_code, pnc::control::TrajectoryCheckBit::TRAJECTORY_UNKNOWN_BIT);
        if (global_paras_.check_result_string_vector.empty()) {
            global_paras_.check_result_string_vector.push_back("fail to show error");
        }
        bflag = false;
    } else {
        // check function
        const pnc::planning::ADCTrajectory* const trajectory_ptr = &control_input_stream.input_stream.planning_info;
        // chassis_ptr being nullptr is included in coming check
        const pnc::chassis::Chassis* const chassis_ptr = &control_input_stream.input_stream.chassis;
        // check kappa
        if (KappaCheck(chassis_ptr, trajectory_ptr, &global_paras_.specific_check_paras) ==
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            LOG_DEBUG("Diagnosis: Bad kappa");
            global_paras_.check_result_string_vector.push_back("bad kappa");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_KAPPA_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_KAPPA_LEVEL_BIT);
            if (control_conf_.diag_v2_conf().has_enable_curvature_check() &&
                !control_conf_.diag_v2_conf().enable_curvature_check()) {
                bflag = bflag && true;
            } else {
                bflag = false;
            }
        }

        // check dkappa
        if (DkappaCheck(chassis_ptr, trajectory_ptr, &global_paras_.specific_check_paras) ==
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            LOG_DEBUG("Diagnosis: Bad d_kappa");
            global_paras_.check_result_string_vector.push_back("bad d_kappa");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_DKAPPA_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_DKAPPA_LEVEL_BIT);
            if (control_conf_.diag_v2_conf().has_enable_curvature_check() &&
                !control_conf_.diag_v2_conf().enable_curvature_check()) {
                bflag = bflag && true;
            } else {
                bflag = false;
            }
        }

        // check theta with xy
        if (XYThetaCheck(trajectory_ptr, &global_paras_.specific_check_paras) ==
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            LOG_DEBUG("Diagnosis: abnormal theta");
            global_paras_.check_result_string_vector.push_back("theta contradict xy");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_THETA_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_THETA_LEVEL_BIT);
            if (control_conf_.diag_v2_conf().has_enable_trajectory_xytheta_check() &&
                !control_conf_.diag_v2_conf().enable_trajectory_xytheta_check()) {
                bflag = bflag && true;
            } else {
                bflag = false;
            }
        }

        // check longitudinal displacement
        if (LongitudinalCheck(trajectory_ptr, &global_paras_.specific_check_paras) ==
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            LOG_DEBUG("Diagnosis: contradictory longitudinal");
            global_paras_.check_result_string_vector.push_back("contradictory longitudinal");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_LONGITUDINAL_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_LONGITUDINAL_LEVEL_BIT);
            if (control_conf_.diag_v2_conf().has_enable_trajectory_longitudinal_check() &&
                !control_conf_.diag_v2_conf().enable_trajectory_longitudinal_check()) {
                bflag = bflag && true;
            } else {
                bflag = false;
            }
        }

        if (InterframeCheck(trajectory_ptr, &global_paras_.specific_check_paras) ==
            pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE) {
            LOG_DEBUG("Diagnosis: large interframe diff");
            global_paras_.check_result_string_vector.push_back("interframe diff--" +
                                                               global_paras_.specific_check_paras.interframe_check_str);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_INTERFRAME_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_INTERFRAME_LEVEL_BIT);
            if (control_conf_.diag_v2_conf().has_enable_trajectory_interframe_check() &&
                !control_conf_.diag_v2_conf().enable_trajectory_interframe_check()) {
                bflag = bflag && true;
            } else {
                bflag = false;
            }
        }

        // check point size >0 and < thrshhold
        if (trajectory_ptr->trajectory_point_size() > 0 &&
            trajectory_ptr->trajectory_point_size() <
                control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_point_size_threshold()) {
            global_paras_.check_result_string_vector.push_back("point size < threshold");
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_POINT_SIZE_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_POINT_SIZE_LEVEL_BIT);
            LOG_DEBUG("Diagnosis: Trajectory has less than %d points",
                      control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_point_size_threshold());
            bflag = false;
        }

        // postprocess
        if (!global_paras_.check_result_string_vector.empty()) {
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_BIT);
            SetCodeBitHigh(&global_paras_.trajectory_check_code,
                           pnc::control::TrajectoryCheckBit::TRAJECTORY_SUMMARY_LEVEL_BIT);
            SetCodeBitLow(&global_paras_.trajectory_check_code,
                          pnc::control::TrajectoryCheckBit::TRAJECTORY_UNKNOWN_BIT);
        }
    }

    UpdateCodeQueue(&global_paras_);

    SetDebugInfo(global_paras_, diag_debug_ptr);

    return bflag;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::KappaCheck(const pnc::chassis::Chassis* const chassis_ptr,
                                                              const pnc::planning::ADCTrajectory* const trajectory_ptr,
                                                              SpecificCheckParas* const specific_check_paras_ptr) {
    // check pointer
    if (chassis_ptr == nullptr || trajectory_ptr == nullptr || specific_check_paras_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: chassis_ptr or trajectory_ptr or specific_check_paras_ptr is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    // check curvature: ensure not greater than (physical limit = 1.0 / Min_Turn_Radius)
    bool bflag_add_bad_kappa_count = true;
    const double kMinTurnRadius = std::max(veh_paras_.min_turn_radius(), 1e-3);
    const double kCurvaturePhysicalLimit = 1.0 / kMinTurnRadius;
    const double kSpeedLimit = 0.27;
    for (int i = 0; i < trajectory_ptr->trajectory_point_size(); ++i) {
        if (!trajectory_ptr->trajectory_point()[i].path_point().has_kappa()) {
            LOG_DEBUG("Diagnosis: trajectory point %d has no kappa", i);
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        }
        if (std::fabs(trajectory_ptr->trajectory_point()[i].path_point().kappa()) > kCurvaturePhysicalLimit) {
            if (bflag_add_bad_kappa_count && chassis_ptr->has_speed_mps() && chassis_ptr->speed_mps() > kSpeedLimit) {
                bflag_add_bad_kappa_count = false;
                (specific_check_paras_ptr->count_bad_kappa)++;
                // Note: Avoid (specific_check_paras_ptr->count_bad_kappa) overflow
                const int kCountBadKappaLimit = 10000;
                specific_check_paras_ptr->count_bad_kappa =
                    std::min(specific_check_paras_ptr->count_bad_kappa, kCountBadKappaLimit);
                if (FLAGS_enable_control_info_print) {
                    LOG_INFO("ControlDiagnosi: count_bad_kappa: %d", specific_check_paras_ptr->count_bad_kappa);
                }
            }

            std::string check_result_str =
                "Bad kappa: " + std::to_string(trajectory_ptr->trajectory_point()[i].path_point().kappa()) +
                " greater than limit: " + std::to_string(kCurvaturePhysicalLimit);
            check_result_str = "Diagnosis: " + check_result_str;
            LOG_DEBUG(check_result_str);
            check_result_str = "Diagnosis: Trajectory info, timestamp_sec: " +
                               std::to_string(trajectory_ptr->header().timestamp_sec()) +
                               ", point index: " + std::to_string(i);
            if (FLAGS_enable_control_info_print) {
                LOG_INFO(check_result_str);
            }
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::DkappaCheck(const pnc::chassis::Chassis* const chassis_ptr,
                                                               const pnc::planning::ADCTrajectory* const trajectory_ptr,
                                                               SpecificCheckParas* const specific_check_paras_ptr) {
    // check pointer
    if (chassis_ptr == nullptr || trajectory_ptr == nullptr || specific_check_paras_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: chassis_ptr or trajectory_ptr or specific_check_paras_ptr  is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    // check curvature change rate between two adjacent points: ensure not greater than steer_angle_rate
    // steer_wheel_angle = wheel_base * curvature * steer_ratio
    // where, wheel_base && steer_ratio are constant
    // Taking the Derivative of Both Sides of the Equation
    // steer_wheel_angle_change_rate = (wheel_base * steer_ratio) * curvature_change_rate
    bool bflag_add_bad_dkappa_count = true;
    double curvature_to_steer = veh_paras_.wheel_base() * veh_paras_.steer_ratio();
    curvature_to_steer = std::max(curvature_to_steer, 1e-3);
    const double kCurvatureChangeRatePhysicalLimit =
        veh_paras_.max_steer_angle_rate() * 180.0 / M_PI / curvature_to_steer;
    // Note: The minimun of trajectory point must be greater than 2
    int kPointSizeLimt = 2;
    if (trajectory_ptr->trajectory_point_size() < kPointSizeLimt) {
        LOG_DEBUG("Diagnosis: trajectory_point_size less than %d", kPointSizeLimt);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    if (!trajectory_ptr->trajectory_point()[0].path_point().has_kappa()) {
        LOG_DEBUG("Diagnosis: trajectory point 0 has no kappa");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    for (int i = 1; i < trajectory_ptr->trajectory_point_size(); ++i) {
        if (!trajectory_ptr->trajectory_point()[i].path_point().has_kappa()) {
            LOG_DEBUG("Diagnosis: trajectory point %d has no kappa", i);
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        }
        double diff_curvature = trajectory_ptr->trajectory_point(i).path_point().kappa() -
                                trajectory_ptr->trajectory_point(i - 1).path_point().kappa();
        // Note: diff_time_sec is positive, which has been checked in function RelativeTimeCheck
        double diff_time_sec = trajectory_ptr->trajectory_point(i).relative_time() -
                               trajectory_ptr->trajectory_point(i - 1).relative_time();
        double curvature_change_rate = diff_curvature / diff_time_sec;
        const double kSpeedLimit = 0.27;
        if (std::fabs(curvature_change_rate) > kCurvatureChangeRatePhysicalLimit) {
            if (bflag_add_bad_dkappa_count && chassis_ptr->has_speed_mps() && chassis_ptr->speed_mps() > kSpeedLimit) {
                bflag_add_bad_dkappa_count = false;
                (specific_check_paras_ptr->count_bad_dkappa)++;
                // Note: Avoid specific_check_paras_ptr->count_bad_dkappa overflow
                const int kCountBadDkappaLimit = 10000;
                specific_check_paras_ptr->count_bad_dkappa =
                    std::min(specific_check_paras_ptr->count_bad_dkappa, kCountBadDkappaLimit);
                if (FLAGS_enable_control_info_print) {
                    LOG_INFO("Diagnosis: count_bad_dkappa: %d", specific_check_paras_ptr->count_bad_dkappa);
                }
            }

            std::string check_result_str = "Bad dkappa: " + std::to_string(curvature_change_rate) +
                                           " greater than limit: " + std::to_string(kCurvatureChangeRatePhysicalLimit);
            check_result_str = "Diagnosis: " + check_result_str;
            LOG_DEBUG(check_result_str);
            check_result_str = "Diagnosis: Trajectory info, timestamp_sec: " +
                               std::to_string(trajectory_ptr->header().timestamp_sec()) +
                               ", point index: " + std::to_string(i);
            if (FLAGS_enable_control_info_print) {
                LOG_INFO(check_result_str);
            }
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
        }
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::XYThetaCheck(
    const pnc::planning::ADCTrajectory* const trajectory_ptr, SpecificCheckParas* const specific_check_paras_ptr) {
    // check pointer
    if (trajectory_ptr == nullptr || specific_check_paras_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: trajectory_ptr or specific_check_paras_ptr  is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    const int kPointSizeMin = 2;
    const int kPointSizeMax =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_innerframe_check_max_point_size();
    if (static_cast<int>(trajectory_ptr->trajectory_point_size()) < kPointSizeMin) {
        LOG_DEBUG("Diagnosis: trajectory_point_size less than %d", kPointSizeMin);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    if (!trajectory_ptr->trajectory_point()[0].path_point().has_x() ||
        !trajectory_ptr->trajectory_point()[0].path_point().has_y() ||
        !trajectory_ptr->trajectory_point()[0].path_point().has_theta() || !trajectory_ptr->has_gear()) {
        LOG_DEBUG("Diagnosis: trajectory point 0 has no x or y or theta or gear");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    bool is_gear_reverse = trajectory_ptr->gear() == pnc::chassis::GEAR_REVERSE;
    bool bflag_theta_abnormal{false};
    const double kXY2ThetaCheckThreshold =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_xy2theta_check_threshold();
    double pre_calculated_theta{0.0};
    for (int i = 1; i < static_cast<int>(trajectory_ptr->trajectory_point_size()) && i < kPointSizeMax; ++i) {
        if (!trajectory_ptr->trajectory_point(i).path_point().has_x() ||
            !trajectory_ptr->trajectory_point(i).path_point().has_y() ||
            !trajectory_ptr->trajectory_point(i).path_point().has_theta()) {
            LOG_DEBUG("Diagnosis: trajectory point %d has no x or y or theta", i);
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        }
        double last_x = trajectory_ptr->trajectory_point(i - 1).path_point().x();
        double last_y = trajectory_ptr->trajectory_point(i - 1).path_point().y();
        double last_theta = trajectory_ptr->trajectory_point(i - 1).path_point().theta();
        double current_x = trajectory_ptr->trajectory_point(i).path_point().x();
        double current_y = trajectory_ptr->trajectory_point(i).path_point().y();

        double calculated_theta{0.0};
        if (is_gear_reverse) {
            calculated_theta =
                jiduauto::pnc::math::NormalizeAngle(std::atan2(current_y - last_y, current_x - last_x) + M_PI);
        } else {
            calculated_theta = jiduauto::pnc::math::NormalizeAngle(std::atan2(current_y - last_y, current_x - last_x));
        }
        if (std::sqrt(std::pow(current_y - last_y, 2) + std::pow(current_x - last_x, 2)) < 1e-3) {
            if (i > 1) {
                calculated_theta = pre_calculated_theta;
            } else {
                calculated_theta = last_theta;
            }
        }
        pre_calculated_theta = calculated_theta;
        double theta_diff = jiduauto::pnc::math::NormalizeAngle(last_theta - calculated_theta);
        if (std::fabs(theta_diff) > std::fabs(specific_check_paras_ptr->max_theta_diff)) {
            specific_check_paras_ptr->max_theta_diff = theta_diff;
        }
        if (std::fabs(theta_diff) > kXY2ThetaCheckThreshold) {
            bflag_theta_abnormal = true;
            LOG_DEBUG("current_x(%.4f),current_y(%.4f),last_x(%.4f),last_y(%.4f)", current_x, current_y, last_x,
                      last_y);
            LOG_DEBUG("is_gear_reverse(%d),calculated_theta(%.4f),last_theta(%.4f),theta_diff(%.4f)", is_gear_reverse,
                      calculated_theta, last_theta, theta_diff);
        }
    }
    if (bflag_theta_abnormal) {
        if (specific_check_paras_ptr->count_abnormal_theta < INT_MAX) {
            specific_check_paras_ptr->count_abnormal_theta += 1;
        }
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::LongitudinalCheck(
    const pnc::planning::ADCTrajectory* const trajectory_ptr, SpecificCheckParas* const specific_check_paras_ptr) {
    // check pointer
    if (trajectory_ptr == nullptr || specific_check_paras_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: trajectory_ptr or specific_check_paras_ptr is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    const int kPointSizeMin = 2;
    const int kPointSizeMax =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_innerframe_check_max_point_size();

    if (static_cast<int>(trajectory_ptr->trajectory_point_size()) < kPointSizeMin) {
        LOG_DEBUG("Diagnosis: trajectory_point_size less than %d", kPointSizeMin);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    if (!trajectory_ptr->trajectory_point()[0].path_point().has_x() ||
        !trajectory_ptr->trajectory_point()[0].path_point().has_y() ||
        !trajectory_ptr->trajectory_point()[0].path_point().has_theta() || !trajectory_ptr->has_gear()) {
        LOG_DEBUG("Diagnosis: trajectory point 0 has no x or y or theta or gear");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    int gear_coef = trajectory_ptr->gear() == pnc::chassis::GEAR_REVERSE ? -1 : 1;
    double accumulated_v2xy_diff{0.0};
    double accumulated_s{0.0};
    double accumulated_xy2s{0.0};
    double accumulated_v2s{0.0};
    for (int i = 1; i < static_cast<int>(trajectory_ptr->trajectory_point_size()) && i <= kPointSizeMax; ++i) {
        if (!trajectory_ptr->trajectory_point(i).path_point().has_x() ||
            !trajectory_ptr->trajectory_point(i).path_point().has_y() ||
            !trajectory_ptr->trajectory_point(i).path_point().has_theta() ||
            !trajectory_ptr->trajectory_point(i).has_v() || !trajectory_ptr->trajectory_point(i).path_point().has_s() ||
            !trajectory_ptr->trajectory_point(i).has_relative_time()) {
            LOG_DEBUG("Diagnosis: trajectory point %d has no x or y or theta or v or s or relative time", i);
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        }
        double last_x = trajectory_ptr->trajectory_point(i - 1).path_point().x();
        double last_y = trajectory_ptr->trajectory_point(i - 1).path_point().y();
        double last_theta = trajectory_ptr->trajectory_point(i - 1).path_point().theta();
        double last_v = trajectory_ptr->trajectory_point(i - 1).v();
        double last_s = trajectory_ptr->trajectory_point(i - 1).path_point().s();
        double last_relative_time = trajectory_ptr->trajectory_point(i - 1).relative_time();
        double current_x = trajectory_ptr->trajectory_point(i).path_point().x();
        double current_y = trajectory_ptr->trajectory_point(i).path_point().y();
        double current_v = trajectory_ptr->trajectory_point(i).v();
        double current_relative_time = trajectory_ptr->trajectory_point(i).relative_time();
        double current_s = trajectory_ptr->trajectory_point(i).path_point().s();

        double time_step = std::fabs(current_relative_time - last_relative_time);
        double average_v = (last_v + current_v) / 2.0;
        double d_s = current_s - last_s;
        double v2s = average_v * time_step;
        double calculated_x = last_x + gear_coef * std::cos(last_theta) * average_v * time_step;
        double calculated_y = last_y + gear_coef * std::sin(last_theta) * average_v * time_step;
        double xy2s = std::sqrt(std::pow(current_x - last_x, 2) + std::pow(current_y - last_y, 2));
        double xy_diff = std::sqrt(std::pow(current_x - calculated_x, 2) + std::pow(current_y - calculated_y, 2));
        accumulated_v2xy_diff += xy_diff;
        accumulated_s += d_s;
        accumulated_xy2s += xy2s;
        accumulated_v2s += v2s;
    }
    double accumulated_v2s_diff = std::fabs(accumulated_s - accumulated_v2s);
    double accumulated_xy2s_diff = std::fabs(accumulated_s - accumulated_xy2s);
    double max_longitudinal_diff =
        std::fmax(accumulated_v2xy_diff, std::fmax(accumulated_v2s_diff, accumulated_xy2s_diff));
    specific_check_paras_ptr->max_lon_diff = max_longitudinal_diff;

    const double kLonDiffThreshold =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_longitudinal_check_diff_threshold();

    if (max_longitudinal_diff > kLonDiffThreshold) {
        if (specific_check_paras_ptr->count_contradictory_lon < INT_MAX) {
            specific_check_paras_ptr->count_contradictory_lon += 1;
        }
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

pnc::control::DiagnosisReturnCode TrajectoryCheck::InterframeCheck(
    const pnc::planning::ADCTrajectory* const trajectory_ptr, SpecificCheckParas* const specific_check_paras_ptr) {
    // check pointer
    if (trajectory_ptr == nullptr || specific_check_paras_ptr == nullptr) {
        LOG_DEBUG("Diagnosis: trajectory_ptr or specific_check_paras_ptr is nullptr");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }
    const int kPointSizeMin = 2;
    const int kPointSizeMax =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_interframe_check_max_point_size();
    if (static_cast<int>(trajectory_ptr->trajectory_point_size()) < kPointSizeMin) {
        LOG_DEBUG("Diagnosis: trajectory_point_size less than %d", kPointSizeMin);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    // check trajectory members
    if (!trajectory_ptr->has_is_replan()) {
        LOG_DEBUG("Diagnosis: trajectory has no is_replan");
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    for (int i = 0; i < static_cast<int>(trajectory_ptr->trajectory_point_size()); ++i) {
        if (!trajectory_ptr->trajectory_point(i).path_point().has_x() ||
            !trajectory_ptr->trajectory_point(i).path_point().has_y() ||
            !trajectory_ptr->trajectory_point(i).path_point().has_theta() ||
            !trajectory_ptr->trajectory_point(i).has_v() || !trajectory_ptr->trajectory_point(i).has_a() ||
            !trajectory_ptr->trajectory_point(i).path_point().has_kappa() ||
            !trajectory_ptr->trajectory_point(i).has_relative_time()) {
            LOG_DEBUG("Diagnosis: trajectory point %d has no x or y or v or theta or a or kappa or relative time", i);
            return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
        }
    }

    if (global_paras_.pre_trajectory_ptr == nullptr ||
        static_cast<int>(global_paras_.pre_trajectory_ptr->trajectory_point_size()) < 1) {
        LOG_DEBUG("Diagnosis: pre_trajectory_ptr is nullptr or no point");
        global_paras_.pre_trajectory_ptr->CopyFrom(*trajectory_ptr);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    if (trajectory_ptr->is_replan() &&
        !control_conf_.diag_v2_conf().trajectory_check_conf().enable_interframe_check_if_replan()) {
        LOG_DEBUG("Diagnosis:trajectory is_replan is true, disable interframe check");
        global_paras_.pre_trajectory_ptr->CopyFrom(*trajectory_ptr);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    double pre_header_time = global_paras_.pre_trajectory_ptr->header().timestamp_sec();
    const double kHeaderTimeDiffThr = 0.01;
    if (std::fabs(pre_header_time - trajectory_ptr->header().timestamp_sec()) < kHeaderTimeDiffThr) {
        LOG_DEBUG("Diagnosis:trajectory header time is same as pre_trajectory_ptr, not do interframe check");
        global_paras_.pre_trajectory_ptr->CopyFrom(*trajectory_ptr);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    // interpolate the last trajectroy and match by time
    Interpolation1D::DataType t_x_vec;
    Interpolation1D::DataType t_y_vec;
    InterpolationAngle::DataType t_theta_vec;
    Interpolation1D::DataType t_v_vec;
    Interpolation1D::DataType t_a_vec;
    Interpolation1D::DataType t_kappa_vec;

    for (int i = 0; i < static_cast<int>(global_paras_.pre_trajectory_ptr->trajectory_point_size()); ++i) {
        auto resample_point = global_paras_.pre_trajectory_ptr->trajectory_point(i);
        double resample_time = pre_header_time + resample_point.relative_time();
        t_x_vec.emplace_back(std::make_pair(resample_time, resample_point.path_point().x()));
        t_y_vec.emplace_back(std::make_pair(resample_time, resample_point.path_point().y()));
        t_theta_vec.emplace_back(std::make_pair(resample_time, resample_point.path_point().theta()));
        t_v_vec.emplace_back(std::make_pair(resample_time, resample_point.v()));
        t_a_vec.emplace_back(std::make_pair(resample_time, resample_point.a()));
        t_kappa_vec.emplace_back(std::make_pair(resample_time, resample_point.path_point().kappa()));
    }

    bool fail_to_init_interpolation{false};
    std::unique_ptr<Interpolation1D> t_x_interpolation = std::make_unique<Interpolation1D>();
    if (!t_x_interpolation->Init(t_x_vec)) {
        LOG_WARN("Fail to init time and x table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<Interpolation1D> t_y_interpolation = std::make_unique<Interpolation1D>();
    if (!t_y_interpolation->Init(t_y_vec)) {
        LOG_WARN("Fail to init time and y table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<InterpolationAngle> t_theta_interpolation = std::make_unique<InterpolationAngle>();
    if (!t_theta_interpolation->Init(t_theta_vec)) {
        LOG_WARN("Fail to init time and theta table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<Interpolation1D> t_v_interpolation = std::make_unique<Interpolation1D>();
    if (!t_v_interpolation->Init(t_v_vec)) {
        LOG_WARN("Fail to init time and v table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<Interpolation1D> t_a_interpolation = std::make_unique<Interpolation1D>();
    if (!t_a_interpolation->Init(t_a_vec)) {
        LOG_WARN("Fail to init time and a table");
        fail_to_init_interpolation = true;
    }
    std::unique_ptr<Interpolation1D> t_kappa_interpolation = std::make_unique<Interpolation1D>();
    if (!t_kappa_interpolation->Init(t_kappa_vec)) {
        LOG_WARN("Fail to init time and kappa table");
        fail_to_init_interpolation = true;
    }
    if (fail_to_init_interpolation) {
        LOG_WARN("Diagnosis:Fail to init time interpolation table");
        global_paras_.pre_trajectory_ptr->CopyFrom(*trajectory_ptr);
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_SKIP;
    }

    double interframe_xy_diff{0.0}, interframe_theta_diff{0.0}, interframe_v_diff{0.0}, interframe_a_diff{0.0},
        interframe_kappa_diff{0.0};
    double max_xy_diff{0.0}, max_theta_diff{0.0}, max_v_diff{0.0}, max_a_diff{0.0}, max_kappa_diff{0.0};
    const double kMaxXYThreshold =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_interframe_check_max_xy_threshold();
    const double kMaxThetaThreshold =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_interframe_check_max_theta_threshold();
    const double kMaxVThreshold =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_interframe_check_max_v_threshold();
    const double kMaxAThreshold =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_interframe_check_max_a_threshold();
    const double kMaxKappaThreshold =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_interframe_check_max_kappa_threshold();

    for (int i = 0; i < static_cast<int>(trajectory_ptr->trajectory_point_size()) && i < kPointSizeMax; ++i) {
        auto trajectory_point = trajectory_ptr->trajectory_point(i);
        double match_time = trajectory_ptr->header().timestamp_sec() + trajectory_point.relative_time();
        double pre_max_time = global_paras_.pre_trajectory_ptr->header().timestamp_sec() +
                              global_paras_.pre_trajectory_ptr
                                  ->trajectory_point(global_paras_.pre_trajectory_ptr->trajectory_point_size() - 1)
                                  .relative_time();
        if (match_time > pre_max_time) {
            break;
        }
        double match_x = t_x_interpolation->LinearInterpolate(match_time);
        double match_y = t_y_interpolation->LinearInterpolate(match_time);
        double match_theta = t_theta_interpolation->LinearInterpolate(match_time);
        double match_v = t_v_interpolation->LinearInterpolate(match_time);
        double match_a = t_a_interpolation->LinearInterpolate(match_time);
        double match_kappa = t_kappa_interpolation->LinearInterpolate(match_time);
        interframe_xy_diff = std::sqrt(std::pow(trajectory_point.path_point().x() - match_x, 2) +
                                       std::pow(trajectory_point.path_point().y() - match_y, 2));
        interframe_theta_diff =
            jiduauto::pnc::math::NormalizeAngle(trajectory_point.path_point().theta() - match_theta);
        interframe_v_diff = trajectory_point.v() - match_v;
        interframe_a_diff = trajectory_point.a() - match_a;
        interframe_kappa_diff = trajectory_point.path_point().kappa() - match_kappa;
        max_xy_diff = std::fmax(max_xy_diff, interframe_xy_diff);
        max_theta_diff =
            std::fabs(max_theta_diff) > std::fabs(interframe_theta_diff) ? max_theta_diff : interframe_theta_diff;
        max_v_diff = std::fabs(max_v_diff) > std::fabs(interframe_v_diff) ? max_v_diff : interframe_v_diff;
        max_a_diff = std::fabs(max_a_diff) > std::fabs(interframe_a_diff) ? max_a_diff : interframe_a_diff;
        max_kappa_diff =
            std::fabs(max_kappa_diff) > std::fabs(interframe_kappa_diff) ? max_kappa_diff : interframe_kappa_diff;
    }
    specific_check_paras_ptr->max_interframe_xy_diff = max_xy_diff;
    specific_check_paras_ptr->max_interframe_theta_diff = max_theta_diff;
    specific_check_paras_ptr->max_interframe_v_diff = max_v_diff;
    specific_check_paras_ptr->max_interframe_a_diff = max_a_diff;
    specific_check_paras_ptr->max_interframe_kappa_diff = max_kappa_diff;

    std::string interframe_check_str{""};
    if (std::fabs(max_xy_diff) > kMaxXYThreshold) {
        interframe_check_str.append("xy ");
    }
    if (std::fabs(max_theta_diff) > kMaxThetaThreshold) {
        interframe_check_str.append("theta ");
    }
    if (std::fabs(max_v_diff) > kMaxVThreshold) {
        interframe_check_str.append("v ");
    }
    if (std::fabs(max_a_diff) > kMaxAThreshold) {
        interframe_check_str.append("a ");
    }
    if (std::fabs(max_kappa_diff) > kMaxKappaThreshold) {
        interframe_check_str.append("kappa");
    }
    specific_check_paras_ptr->interframe_check_str = interframe_check_str;
    if (!interframe_check_str.empty()) {
        std::string log_interframe_check_str = "Diagnosis:interframe diff in: " + interframe_check_str;
        if (FLAGS_enable_control_info_print) {
            LOG_INFO(log_interframe_check_str);
        }
    }

    LOG_DEBUG("pre copyfrom cur");
    global_paras_.pre_trajectory_ptr->CopyFrom(*trajectory_ptr);

    if (std::fabs(max_xy_diff) > kMaxXYThreshold || std::fabs(max_theta_diff) > kMaxThetaThreshold ||
        std::fabs(max_v_diff) > kMaxVThreshold || std::fabs(max_a_diff) > kMaxAThreshold ||
        std::fabs(max_kappa_diff) > kMaxKappaThreshold) {
        return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_FALSE;
    }
    return pnc::control::DiagnosisReturnCode::DIAGNOSIS_RETURN_TRUE;
}

void TrajectoryCheck::SetDebugInfo(const GlobalParas& global_param,
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

    diag_debug_ptr->set_trajectory_check_string(check_string);
    diag_debug_ptr->mutable_curvature_check_debug()->set_count_bad_kappa(
        global_param.specific_check_paras.count_bad_kappa);
    diag_debug_ptr->mutable_curvature_check_debug()->set_count_bad_dkappa(
        global_param.specific_check_paras.count_bad_dkappa);
    diag_debug_ptr->mutable_theta_check_debug()->set_max_theta_diff(global_param.specific_check_paras.max_theta_diff);
    diag_debug_ptr->mutable_theta_check_debug()->set_count_abnormal_theta(
        global_param.specific_check_paras.count_abnormal_theta);
    diag_debug_ptr->mutable_trajectory_longitudinal_check_debug()->set_lon_diff(
        global_param.specific_check_paras.max_lon_diff);
    diag_debug_ptr->mutable_trajectory_longitudinal_check_debug()->set_count_contradictory_lon(
        global_param.specific_check_paras.count_contradictory_lon);
    diag_debug_ptr->mutable_interframe_check_debug()->set_max_interframe_xy_diff(
        global_param.specific_check_paras.max_interframe_xy_diff);
    diag_debug_ptr->mutable_interframe_check_debug()->set_max_interframe_theta_diff(
        global_param.specific_check_paras.max_interframe_theta_diff);
    diag_debug_ptr->mutable_interframe_check_debug()->set_max_interframe_v_diff(
        global_param.specific_check_paras.max_interframe_v_diff);
    diag_debug_ptr->mutable_interframe_check_debug()->set_max_interframe_a_diff(
        global_param.specific_check_paras.max_interframe_a_diff);
    diag_debug_ptr->mutable_interframe_check_debug()->set_max_interframe_kappa_diff(
        global_param.specific_check_paras.max_interframe_kappa_diff);
    diag_debug_ptr->set_trajectory_check_code(global_param.trajectory_check_code);
}

void TrajectoryCheck::UpdateCodeQueue(GlobalParas* const global_param) {
    const int kCheckDuration =
        control_conf_.diag_v2_conf().trajectory_check_conf().trajectory_continuous_check_duration();
    global_param->trajectory_check_code_queue.push(global_param->trajectory_check_code);

    while (static_cast<int>(global_param->trajectory_check_code_queue.size()) > kCheckDuration) {
        global_param->trajectory_check_code_queue.pop();
    }
}

}  // namespace control
}  // namespace jiduauto
