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

#include "control/src/common/control_gflags.h"

// -------------config file------------------
DEFINE_string(control_node_name, "control", "control node name.");
DEFINE_string(control_conf_path, "conf/", "The default path of data");
DEFINE_string(vehicle_conf_file, "vel_config.pb.txt", "the vehicle param config file.");
DEFINE_string(control_conf_file, "control_conf.pb.txt", "control conf file name.");
DEFINE_string(control_gflag_file, "conf/control.conf", "gflag config file, cannot change");
DEFINE_string(control_chassis_test_file, "control_chassis_test.pb.txt", "chassis test conf file name.");
DEFINE_string(control_unit_test_data_path, "data/control/test/", "control unit tests data path");
DEFINE_string(control_unit_test_conf_file, "control_conf.pb.txt", "control unit tests control conf file name.");

// -------------channel name------------------
DEFINE_bool(control_enable_debug_channel, false, "true to enable");
DEFINE_string(JIDL_imu_data_topic, "Apollo/DemoTopic", "imu_data channel name");

// -------------frequency parameters------------------
DEFINE_int32(control_list_max_size, 10, "topic list max size");
DEFINE_double(min_control_interval_sec, 0.005, "min control interval sec");

// -------------functions------------------
DEFINE_bool(enable_lowspeed_steer_protect, false, "use pre_steer_angle");
DEFINE_bool(enable_hill_assist_function, false, "true to enable");
DEFINE_bool(enable_big_steer_diff_stop_function, false, "true to stop vehicle during big steer diff");
DEFINE_bool(big_steer_diff_stop_flag, false, "this flag is used for inner state");
DEFINE_double(big_steer_diff_threshold, 10.0, "threshold[%]");
DEFINE_bool(enable_smooth_estop_brake, true, "use smooth brake while estop is true");
DEFINE_bool(enable_UT_diagnosis_depth_test, false,
            "true enable, enable depth test mode in unit_test/test_src/feature/control_diagnosis_test.cc");
DEFINE_bool(enable_control_info_print, false, "enable for printing LOG_INFO");

// -------------micro parameters------------------
DEFINE_bool(query_forward_time_point_only, false, "only use the trajectory point in future");
DEFINE_double(control_stop_acc, 0.001, "acceleration threshold for finding stop index");

// sanity check
DEFINE_bool(enable_lateral_sanity_check, true, "enable lateral state error check to protect vehicle");
DEFINE_bool(enable_lateral_curvature_check, true, "enable flag");
DEFINE_bool(enable_lateral_error_check, true, "enable flag");
DEFINE_bool(enable_heading_error_check, true, "enable flag");
DEFINE_double(low_speed_curvature_threshold, 1.0, "curvature threshold");
DEFINE_double(low_speed_lateral_error_threshold, 6.0, "lateral threshold");
DEFINE_double(low_speed_heading_error_threshold, 1.04, "heading threshold");
DEFINE_double(lateral_curvature_threshold, 1.0, "curvature threshold");
DEFINE_double(lateral_error_threshold, 6.0, "lateral threshold");
DEFINE_double(heading_error_threshold, 1.37, "heading threshold");

DEFINE_double(control_safety_back_slip_speed_thr, -0.15, "speed threshold to check whether the vehicle is slipping");

// add noise
DEFINE_bool(enable_input_noise, false, "enable input noise");
DEFINE_bool(enable_localization_noise_xy, false, "enable noise in xy");
DEFINE_bool(enable_localization_noise_heading, false, "enable in heading");
