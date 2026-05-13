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

#include <gflags/gflags.h>

#include "pnc_common/src/common/pnc_gflag.h"

// -------------config file------------------
DECLARE_string(control_node_name);
DECLARE_string(control_conf_path);
DECLARE_string(vehicle_conf_file);
DECLARE_string(control_conf_file);
DECLARE_string(control_gflag_file);
DECLARE_string(control_chassis_test_file);
DECLARE_string(control_unit_test_data_path);
DECLARE_string(control_unit_test_conf_file);

// -------------channel name------------------
DECLARE_bool(control_enable_debug_channel);
DECLARE_string(JIDL_imu_data_topic);

// -------------frequency parameters------------------
DECLARE_int32(control_list_max_size);
DECLARE_double(min_control_interval_sec);

// -------------functions------------------
DECLARE_bool(enable_lowspeed_steer_protect);
DECLARE_bool(enable_hill_assist_function);
DECLARE_bool(enable_big_steer_diff_stop_function);
DECLARE_bool(big_steer_diff_stop_flag);
DECLARE_double(big_steer_diff_threshold);
DECLARE_bool(enable_smooth_estop_brake);
DECLARE_bool(enable_UT_diagnosis_depth_test);
DECLARE_bool(enable_control_info_print);

// -------------micro parameters------------------
DECLARE_bool(query_forward_time_point_only);
DECLARE_double(control_stop_acc);

// -------------sanity check------------------
DECLARE_bool(enable_lateral_sanity_check);
DECLARE_bool(enable_lateral_curvature_check);
DECLARE_bool(enable_lateral_error_check);
DECLARE_bool(enable_heading_error_check);
DECLARE_double(low_speed_curvature_threshold);
DECLARE_double(low_speed_lateral_error_threshold);
DECLARE_double(low_speed_heading_error_threshold);
DECLARE_double(lateral_curvature_threshold);
DECLARE_double(lateral_error_threshold);
DECLARE_double(heading_error_threshold);
DECLARE_double(control_safety_back_slip_speed_thr);

// ------------------add noise------------------
DECLARE_bool(enable_input_noise);
DECLARE_bool(enable_localization_noise_xy);
DECLARE_bool(enable_localization_noise_heading);
