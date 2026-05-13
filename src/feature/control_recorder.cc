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

#include "control/src/feature/control_recorder.h"

#include <chrono>
#include <limits>
#include <string>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/math/control_math.h"

namespace jiduauto {
namespace control {

void ControlRecorder::LogDebugFile(const ControlCommandStream& control_command_stream,
                                   const pnc::control::ControlConfig& control_conf) {
    if (!(control_log_file_ == nullptr && !InitLogFile(control_command_stream, control_conf))) {
        fprintf(control_log_file_,
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %d, %d, %d, %d, %.8f, %.8f, %.8f, %d, %.8f, %.8f, "
                "%.8f, %.8f, %d, %d, %d, %d, %d, %d, %.8f, %.8f, %d, %d, "
                "%d, %d, %d, %d, %.8f, %.8f, %.8f, %d, %d, %d, %.8f, %.8f, "
                "%.8f, %d, %.8f, %.8f, %d, %.8f, %.8f, %d, %d\n",
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().timestamp_sec(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().x(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().y(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().z(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().roll(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().pitch(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().yaw(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().heading(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().curvature(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().angular_velocity(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().velocity_component_enu().x(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().velocity_component_enu().y(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().velocity_component_enu().z(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().localization_velocity(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().linear_velocity(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().wheelspd_resultant_velocity(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().linear_acceleration(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().steering_angle(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().steering_angle_rate(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().steering_percentage(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().gear(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().driving_mode(),
                control_command_stream.control_command.driving_mode(),
                control_command_stream.control_command.gear_location(),
                control_command_stream.control_command.acceleration(),
                control_command_stream.control_command.steering_target(),
                control_command_stream.control_command.steering_rate(),
                control_command_stream.control_command.pad_msg().action(),
                control_command_stream.control_debug.longitudinal_debug().station_error(),
                control_command_stream.control_debug.longitudinal_debug().speed_error(),
                control_command_stream.control_debug.lateral_debug().lateral_error_by_s(),
                control_command_stream.control_debug.lateral_debug().heading_error_by_s(),
                control_command_stream.control_debug.diagnosis_debug_v2().control_iteration_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .localization_jump_debug()
                    .localization_jump_count(),
                control_command_stream.control_debug.diagnosis_debug_v2().curvature_check_debug().count_bad_kappa(),
                control_command_stream.control_debug.diagnosis_debug_v2().curvature_check_debug().count_bad_dkappa(),
                static_cast<int>(control_command_stream.control_debug.diagnosis_debug_v2().reaction_code()),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .steer_angle_offset_debug()
                    .is_steer_angle_offset_mean_update(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .steer_angle_offset_debug()
                    .steer_angle_offset_after_check(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .steer_angle_offset_debug()
                    .temp_steer_angle_offset(),
                control_command_stream.control_debug.diagnosis_debug_v2().time_cost().pre_process_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2().time_cost().produce_cmd_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2().time_cost().post_process_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2().chassis_check_code(),
                control_command_stream.control_debug.diagnosis_debug_v2().localization_check_code(),
                control_command_stream.control_debug.diagnosis_debug_v2().trajectory_check_code(),
                control_command_stream.control_debug.noise_generator_debug().pure_x(),
                control_command_stream.control_debug.noise_generator_debug().pure_y(),
                control_command_stream.control_debug.noise_generator_debug().pure_yaw(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .acc_pedal_debug()
                    .acc_pedal_abnormal(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().standstill(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().require_standstill_steering(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .localization_jump_debug()
                    .accumulated_distanece_error(),
                control_command_stream.control_debug.longitudinal_debug().station_target(),
                control_command_stream.control_debug.longitudinal_debug().speed_target(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .gear_debug()
                    .gear_abnormal(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().motor_torque(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().motor_speed(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().parking_brake(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().steering_torque(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().control_terminal_swa(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().require_terminal_process(),
                control_command_stream.control_debug.diagnosis_debug_v2().actuator_check_code());

        fflush(control_log_file_);

        if (control_command_stream.control_debug.pid_speed_debug().enable_controller()) {
            fprintf(
                pid_speed_log_file_,
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %d, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, "
                "%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f\n",
                control_command_stream.control_debug.pid_speed_debug().lon_context().timestamp_sec(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().forsee_time(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().forsee_min_distance(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().station_kp(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().station_ki(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().station_kd(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .pid_params()
                    .station_integrator_saturation_level(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().speed_kp(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().speed_ki(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pid_params().speed_kd(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .pid_params()
                    .speed_integrator_saturation_level(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().current_station(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().current_lateral_dis(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().station_target(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().station_error(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_station_target(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_station_error(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().station_loop_input(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().station_loop_output(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().current_speed(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().frenet_lon_speed(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().frenet_lat_speed(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().speed_target(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().speed_error(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_speed_target(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_speed_error(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().speed_loop_input(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().speed_loop_output(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().current_accel(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().frenet_lon_accel(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().frenet_lat_accel(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().accel_target(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().accel_error(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_accel_target(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_accel_error(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().accel_closedloop(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().accel_cmd(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().heading_error(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().pitch_angle(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().slope_offset_compensation(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().path_remain(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().open_loop_time(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().is_near_stop(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().throttle_cmd(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().brake_cmd(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().matched_point().x(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().matched_point().y(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().matched_point().z(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().matched_point().theta(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().matched_point().kappa(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().matched_point().dkappa(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().matched_point().s(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().target_point().path_point().x(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().target_point().path_point().y(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().target_point().path_point().z(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .target_point()
                    .path_point()
                    .theta(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .target_point()
                    .path_point()
                    .kappa(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .target_point()
                    .path_point()
                    .dkappa(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().target_point().path_point().s(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().target_point().v(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().target_point().a(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().target_point().relative_time(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_point().path_point().x(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_point().path_point().y(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_point().path_point().z(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .preview_point()
                    .path_point()
                    .theta(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .preview_point()
                    .path_point()
                    .kappa(),
                control_command_stream.control_debug.pid_speed_debug()
                    .lon_context()
                    .preview_point()
                    .path_point()
                    .dkappa(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_point().path_point().s(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_point().v(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_point().a(),
                control_command_stream.control_debug.pid_speed_debug().lon_context().preview_point().relative_time());
            fflush(pid_speed_log_file_);
        }
    }

    if (log_index_ >= 10) {
        if (FLAGS_enable_control_info_print == true) {
            LOG_INFO(
                "LogDebugInfo: timestamp_sec: %.8f\n"
                "x: %.8f | y: %.8f | z: %.8f\n"
                "roll: %.8f | pitch: %.8f | yaw: %.8f | heading: %.8f\n"
                "curvature: %.8f | angular_velocity: %.8f\n"
                "speed_enu_x: %.8f | speed_enu_y: %.8f | speed_enu_z: %.8f\n"
                "localization_velocity: %.8f | linear_velocity: %.8f | linear_acceleration: %.8f\n"
                "steering_angle: %.8f | steering_angle_rate: %.8f | steering_percentage: %.8f\n"
                "gear: %d | driving_mode: %d | driving_mode_cmd: %d\n"
                "gear_cmd: %d | acceleration_cmd: %.8f | steering_cmd: %.8f\n"
                "steering_rate_cmd: %.8f | action: %d\n"
                "station_error: %.8f | speed_error: %.8f\n"
                "lateral_error: %.8f | heading_error: %.8f\n"
                "control_iteration_cost_ms: %d | localization_jump_count: %d\n"
                "count_bad_kappa: %d | count_bad_dkappa: %d\n"
                "fallback_reaction_index: %d | steer_offset_update_status: %d\n"
                "steer_offset_after_check: %.8f| temp_steer_offset: %.8f\n"
                "pre_process_cost_ms: %d | produce_cmd_cost_ms: %d | post_process_cost_ms: %d\n"
                "chassis_check_code: %d | localization_check_code: %d | trajectory_check_code: %d\n"
                "pure_x: %.8f | pure_y: %.8f | pure_yaw: %.8f | localization_accumulater_error: %.8f",
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().timestamp_sec(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().x(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().y(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().z(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().roll(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().pitch(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().yaw(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().heading(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().curvature(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().angular_velocity(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().velocity_component_enu().x(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().velocity_component_enu().y(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().velocity_component_enu().z(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().localization_velocity(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().linear_velocity(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().linear_acceleration(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().steering_angle(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().steering_angle_rate(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().steering_percentage(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().gear(),
                control_command_stream.control_debug.virtual_sensors_debug().sense_view().driving_mode(),
                control_command_stream.control_command.driving_mode(),
                control_command_stream.control_command.gear_location(),
                control_command_stream.control_command.acceleration(),
                control_command_stream.control_command.steering_target(),
                control_command_stream.control_command.steering_rate(),
                control_command_stream.control_command.pad_msg().action(),
                control_command_stream.control_debug.longitudinal_debug().station_error(),
                control_command_stream.control_debug.longitudinal_debug().speed_error(),
                control_command_stream.control_debug.lateral_debug().lateral_error_by_s(),
                control_command_stream.control_debug.lateral_debug().heading_error_by_s(),
                control_command_stream.control_debug.diagnosis_debug_v2().control_iteration_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .localization_jump_debug()
                    .localization_jump_count(),
                control_command_stream.control_debug.diagnosis_debug_v2().curvature_check_debug().count_bad_kappa(),
                control_command_stream.control_debug.diagnosis_debug_v2().curvature_check_debug().count_bad_dkappa(),
                static_cast<int>(control_command_stream.control_debug.diagnosis_debug_v2().reaction_code()),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .steer_angle_offset_debug()
                    .is_steer_angle_offset_mean_update(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .steer_angle_offset_debug()
                    .steer_angle_offset_after_check(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .acutator_check_debug()
                    .steer_angle_offset_debug()
                    .temp_steer_angle_offset(),
                control_command_stream.control_debug.diagnosis_debug_v2().time_cost().pre_process_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2().time_cost().produce_cmd_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2().time_cost().post_process_cost_ms(),
                control_command_stream.control_debug.diagnosis_debug_v2().chassis_check_code(),
                control_command_stream.control_debug.diagnosis_debug_v2().localization_check_code(),
                control_command_stream.control_debug.diagnosis_debug_v2().trajectory_check_code(),
                control_command_stream.control_debug.noise_generator_debug().pure_x(),
                control_command_stream.control_debug.noise_generator_debug().pure_y(),
                control_command_stream.control_debug.noise_generator_debug().pure_yaw(),
                control_command_stream.control_debug.diagnosis_debug_v2()
                    .localization_jump_debug()
                    .accumulated_distanece_error());
        }
        log_index_ = 0;
    }
    ++log_index_;
}
void ControlRecorder::LogTrajectoryFile(const ControlInputStream& control_input_stream,
                                        const pnc::control::ControlConfig& control_conf) {
    if (control_input_stream.input_stream.planning_info.trajectory_point_size() < 1) {
        LOG_WARN("pnc_trajectory_points_size < 1");
        return;
    }
    //  find the closest point along path trajectory
    //  copy from trajectory_analyzer
    int current_index{0};
    if (control_conf.control_preprocess_conf().log_trajectory_file_conf().enable_logtrajectory_find_nearest_point()) {
        double closet_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < control_input_stream.input_stream.planning_info.trajectory_point_size(); ++i) {
            double curr_dist =
                std::hypot(control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().x() -
                               control_input_stream.input_stream.localization.loc_pose().pose().position().x(),
                           control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().y() -
                               control_input_stream.input_stream.localization.loc_pose().pose().position().y());
            if (curr_dist < closet_dist) {
                closet_dist = curr_dist;
                current_index = i;
            }
        }
    }
    if (!(trajectory_log_file_ == nullptr && !InitTrajectoryFile())) {
        // sample time = N period from proto
        if (control_conf.control_preprocess_conf().log_trajectory_file_conf().trajectory_log_period() == 0) {
            LOG_WARN("trajectory_log_period = 0 !");
        } else if (log_count_ %
                       control_conf.control_preprocess_conf().log_trajectory_file_conf().trajectory_log_period() ==
                   0) {
            LOG_INFO("trajectory_log_period = %d , trajectory_log_length = %d",
                     control_conf.control_preprocess_conf().log_trajectory_file_conf().trajectory_log_period(),
                     control_conf.control_preprocess_conf().log_trajectory_file_conf().trajectory_log_length());
            ++trajectory_log_index_;
            for (int i = current_index;
                 i < current_index +
                         control_conf.control_preprocess_conf().log_trajectory_file_conf().trajectory_log_length() &&
                 i < control_input_stream.input_stream.planning_info.trajectory_point_size();
                 ++i) {
                fprintf(trajectory_log_file_,
                        "%d, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %d, %d\n",
                        trajectory_log_index_,
                        control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().x(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().y(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().z(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().theta(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().kappa(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().dkappa(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).v(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).a(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).relative_time(),
                        control_input_stream.input_stream.planning_info.header().timestamp_sec(),
                        control_input_stream.input_stream.planning_info.trajectory_point(i).path_point().s(),
                        control_input_stream.input_stream.planning_info.gear(),
                        control_input_stream.input_stream.planning_info.is_replan());
                fflush(trajectory_log_file_);
            }
            int last_point_index = control_input_stream.input_stream.planning_info.trajectory_point_size() - 1;
            fprintf(
                trajectory_log_file_, "%d, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %d, %d\n",
                trajectory_log_index_,
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).path_point().x(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).path_point().y(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).path_point().z(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).path_point().theta(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).path_point().kappa(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index)
                    .path_point()
                    .dkappa(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).v(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).a(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).relative_time(),
                control_input_stream.input_stream.planning_info.header().timestamp_sec(),
                control_input_stream.input_stream.planning_info.trajectory_point(last_point_index).path_point().s(),
                control_input_stream.input_stream.planning_info.gear(),
                control_input_stream.input_stream.planning_info.is_replan());
            fflush(trajectory_log_file_);
        }
    }
    ++log_count_;
}

bool ControlRecorder::InitLogFile(const ControlCommandStream& control_command_stream,
                                  const pnc::control::ControlConfig& control_conf) {
    auto now_time = std::chrono::system_clock::now();
    auto now_time_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now_time);
    std::time_t now_c = std::chrono::system_clock::to_time_t(now_time_seconds);
    char name_buffer[80];
    strftime(name_buffer, 80, "vehicle_state_and_command_log_%Y%m%d%H%M%S.csv", std::localtime(&now_c));

    std::string file_name = FLAGS_pnc_data_log_path + "/control/" + name_buffer;
    control_log_file_ = fopen(file_name.c_str(), "w");
    if (control_log_file_ == nullptr) {
        LOG_ERROR("control log file record failed.");
        return false;
    }
    fprintf(control_log_file_,
            "timestamp_sec, raw_x, raw_y, raw_z, raw_roll, raw_pitch, raw_yaw, raw_heading, curvature, "
            "angular_velocity, speed_enu_x, speed_enu_y, speed_enu_z, localization_velocity, "
            "linear_velocity, whlspd_resultant_velocity, linear_acceleration, steering_angle, steering_angle_spd, "
            "steering_percentage, gear, driving_mode, driving_mode_cmd, gear_cmd, "
            "acceleration_cmd, steering_cmd, steering_rate_cmd, driving_action, station_error, speed_error, "
            "lateral_error, heading_error, control_iteration_cost_ms, localization_jump_count, count_bad_kappa, "
            "count_bad_dkappa, fallback_reaction_index, steer_offset_update_status, "
            "steer_offset_after_check, temp_steer_offset, pre_process_cost_ms, produce_cmd_cost_ms, "
            "post_process_cost_ms, chassis_check_code, localization_check_code, trajectory_check_code, pure_x, pure_y, "
            "pure_yaw, acc_pedal_abnormal, standstill, require_standstill_steering, accumulated_error, station_target, "
            "speed_target, gear_abnormal, motor_torque, motor_speed, parking_brake, steering_torque, "
            "control_terminal_swa, require_terminal_process, actuator_check_code\n");

    if (control_conf.controller_config().lon_init() == pnc::control::ControlConfig::PID_SPEED_CONTROLLER) {
        std::memset(name_buffer, 0, sizeof(name_buffer));
        strftime(name_buffer, 80, "pid_speed_log_%Y%m%d%H%M%S.csv", std::localtime(&now_c));
        std::string file_name = FLAGS_pnc_data_log_path + "/control/" + name_buffer;
        pid_speed_log_file_ = fopen(file_name.c_str(), "w");
        if (pid_speed_log_file_ == nullptr) {
            LOG_ERROR("pid speed log file record failed.");
            return false;
        }
        fprintf(pid_speed_log_file_,
                "timestamp_sec, preview_time, preview_distance, station_kp, station_ki, station_kd, "
                "station_integrator_saturation_level, speed_kp, speed_ki, speed_kd, "
                "speed_integrator_saturation_level, "
                "current_station, current_lat_distance, station_target, station_error, "
                "preview_station_target, "
                "preview_station_error, station_loop_input, station_loop_output, current_speed, "
                "frenet_lon_speed, "
                "frenet_lat_speed, speed_target, preview_speed_target, preview_speed_error, "
                "speed_loop_input, "
                "speed_loop_output, current_accel, frenet_lon_accel, frenet_lat_accel, accel_target, "
                "accel_error, "
                "preview_accel_target, preview_accel_error, accel_closedloop, accel_cmd, heading_error, "
                "pitch_angle, "
                "slope_offset_compensation, path_remain, open_loop_time, is_near_stop, throttle_cmd, "
                "brake_cmd, "
                "matched_point.x, matched_point.y, matched_point.z, matched_point.theta, "
                "matched_point.kappa, "
                "matched_point.dkappa, matched_point.s, target_point.x, target_point.y, target_point.z, "
                "target_point.theta, target_point.kappa, target_point.dkappa, target_point.s, "
                "target_point.v, "
                "target_point.a, target_point.relative_time, preview_point.x, preview_point.y, "
                "preview_point.z, "
                "preview_point.theta, preview_point.kappa, preview_point.dkappa, preview_point.s, "
                "preview_point.v, "
                "preview_point.a, preview_point.relative_time, cost_time_ms\n");
    }

    return true;
}

bool ControlRecorder::InitTrajectoryFile() {
    auto now_time = std::chrono::system_clock::now();
    auto now_time_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now_time);
    std::time_t now_c = std::chrono::system_clock::to_time_t(now_time_seconds);
    char name_buffer[80];
    strftime(name_buffer, 80, "trajectory_log_%Y%m%d%H%M%S.csv", std::localtime(&now_c));

    std::string trajectory_file_name = FLAGS_pnc_data_log_path + "/control/" + name_buffer;
    trajectory_log_file_ = fopen(trajectory_file_name.c_str(), "w");
    if (trajectory_log_file_ == nullptr) {
        return false;
    }
    fprintf(trajectory_log_file_,
            "index, x, y, z, theta, kappa, dkappa, v, a, relative_time, header_time, s, gear, is_replan\n");
    return true;
}

}  // namespace control
}  // namespace jiduauto
