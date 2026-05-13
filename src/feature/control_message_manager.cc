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

#include "control/src/feature/control_message_manager.h"

#include "control/src/common/control_gflags.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/utility/time_utility.h"

namespace jiduauto {
namespace control {

using jiduauto::pnc::util::TimeUtility;

/// Get all latest message for control
const InputStream ControlMessageManager::GetOnboardMessage() {
    InputStream input_stream;
    TrajectoryConstPtr planning_ptr = GetLatestPlanningMessage();
    if (planning_ptr != nullptr) {
        input_stream.planning_info = *planning_ptr;
    }
    ChassisConstPtr chassis_ptr = GetLatestChassisMessage();
    if (chassis_ptr != nullptr) {
        input_stream.chassis = *chassis_ptr;
    }
    LocalizationConstPtr localization_ptr = GetLatestLocalizationMessage();
    if (localization_ptr != nullptr) {
        input_stream.localization = *localization_ptr;
    }
    ControlCommandConstPtr control_command_ptr = GetLatestControlMessage();
    if (control_command_ptr != nullptr) {
        input_stream.control_command = *control_command_ptr;
    }

    input_stream.pad_info = GetLatestPadMessage();

    return input_stream;
}

void ControlMessageManager::SaveChassisMessage(const ChassisConstPtr& chassis_ptr) {
    if (chassis_ptr == nullptr) {
        LOG_ERROR("chassis check failed!");
        return;
    }
    auto chassis_msg = std::make_shared<pnc::chassis::Chassis>();
    chassis_msg->CopyFrom(*chassis_ptr);

    if (FLAGS_enable_logsim_mode) {
        chassis_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
    }

    if (chassis_msg->has_header()) {
        double chassis_time_delay = TimeUtility::GetCurrentTimesecond() - chassis_msg->header().timestamp_sec();
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("chassis_time_delay: %.6f, chassis_seq: %d", chassis_time_delay,
                     chassis_msg->header().sequence_num());
        }
        if (chassis_time_delay < -0.001) {
            chassis_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
            LOG_ERROR("chassis_time_behind: %.6f, chassis_seq: %d", chassis_time_delay,
                      chassis_msg->header().sequence_num());
        }
    } else {
        LOG_ERROR("no timestamp in chassis msg");
        return;
    }
    // add by chi. check whether same msg
    if (!chassis_list_.empty()) {
        if (std::abs(chassis_msg->header().timestamp_sec() - chassis_list_.back()->header().timestamp_sec()) < 1e-2 &&
            chassis_msg->header().sequence_num() == chassis_list_.back()->header().sequence_num()) {
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(chassis_mutex_);
        chassis_list_.push_back(chassis_msg);
        if (chassis_list_.size() > FLAGS_control_list_max_size) {
            chassis_list_.pop_front();
        }
    }
    return;
}

ChassisConstPtr ControlMessageManager::GetLatestChassisMessage() {
    std::lock_guard<std::mutex> lock(chassis_mutex_);
    if (!chassis_list_.empty()) {
        return chassis_list_.back();
    } else {
        LOG_INFO("chassis list is empty");
        return nullptr;
    }
}

void ControlMessageManager::SavePlanningMessage(const TrajectoryConstPtr& planning_ptr) {
    if (planning_ptr == nullptr) {
        LOG_ERROR("trajectory check failed!");
        return;
    }

    auto trajectory_msg = std::make_shared<pnc::planning::ADCTrajectory>();
    trajectory_msg->CopyFrom(*planning_ptr);

    if (FLAGS_enable_logsim_mode) {
        trajectory_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
    }

    if (trajectory_msg->has_header()) {
        double traj_time_delay = TimeUtility::GetCurrentTimesecond() - trajectory_msg->header().timestamp_sec();
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("planning_time_delay: %.6f, planning_seq: %d", traj_time_delay,
                     trajectory_msg->header().sequence_num());
        }
        if (traj_time_delay < -0.001) {
            trajectory_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
            LOG_ERROR("planning_time_behind: %.6f, planning_seq: %d", traj_time_delay,
                      trajectory_msg->header().sequence_num());
        }
    } else {
        LOG_ERROR("no timestamp in planning msg");
        return;
    }
    // add by chi. check whether same msg
    if (!planning_list_.empty()) {
        if (std::abs(trajectory_msg->header().timestamp_sec() - planning_list_.back()->header().timestamp_sec()) <
                1e-2 &&
            trajectory_msg->header().sequence_num() == planning_list_.back()->header().sequence_num()) {
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        planning_list_.push_back(trajectory_msg);
        if (planning_list_.size() > FLAGS_control_list_max_size) {
            planning_list_.pop_front();
        }
    }
    return;
}

TrajectoryConstPtr ControlMessageManager::GetLatestPlanningMessage() {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    if (!planning_list_.empty()) {
        return planning_list_.back();
    } else {
        LOG_INFO("planning message list is empty");
        return nullptr;
    }
}

void ControlMessageManager::SaveLocalizationMessage(const LocalizationConstPtr& localization_ptr) {
    if (localization_ptr == nullptr) {
        LOG_ERROR("localization_ptr check failed!");
        return;
    }

    auto localization_msg = std::make_shared<pnc::localization::LocalizationEstimate>();
    localization_msg->CopyFrom(*localization_ptr);

    if (FLAGS_enable_logsim_mode) {
        localization_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
    }
    if (!FLAGS_enable_using_global_localization) {
        auto pose = localization_msg->loc_pose().pose();
        localization_msg->mutable_loc_pose()->mutable_pose()->CopyFrom(localization_msg->loc_pose().local_pose());
        localization_msg->mutable_loc_pose()->mutable_local_pose()->CopyFrom(pose);
    }

    if (localization_msg->has_header()) {
        double localization_time_delay =
            TimeUtility::GetCurrentTimesecond() - localization_msg->header().timestamp_sec();
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("localization_time_delay: %.6f, localization_seq: %d", localization_time_delay,
                     localization_msg->header().sequence_num());
        }
        if (localization_time_delay < -0.001) {
            localization_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
            LOG_ERROR("localization_time_behind: %.6f, localization_seq: %d", localization_time_delay,
                      localization_msg->header().sequence_num());
        }
    } else {
        LOG_ERROR("no timestamp in localizaiton msg");
        return;
    }

    // add by chi. check whether same msg
    if (!localization_list_.empty()) {
        if (std::abs(localization_msg->header().timestamp_sec() - localization_list_.back()->header().timestamp_sec()) <
                1e-2 &&
            localization_msg->header().sequence_num() == localization_list_.back()->header().sequence_num()) {
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(localization_mutex_);
        localization_list_.push_back(localization_msg);
        if (localization_list_.size() > FLAGS_control_list_max_size) {
            localization_list_.pop_front();
        }
    }

    return;
}

LocalizationConstPtr ControlMessageManager::GetLatestLocalizationMessage() {
    std::lock_guard<std::mutex> lock(localization_mutex_);
    if (!localization_list_.empty()) {
        return localization_list_.back();
    } else {
        LOG_INFO("localization list is empty");
        return nullptr;
    }
}

void ControlMessageManager::SavePadMessage(const PadMessageConstPtr& pad_message_ptr) {
    if (pad_message_ptr == nullptr) {
        LOG_ERROR("PadMessage sanity check failed!");
        return;
    }

    pnc::control::PadMessage pad_message = *pad_message_ptr;
    {
        std::lock_guard<std::mutex> lock(pad_message_mutex_);
        pad_message_list_.push_back(pad_message);
        if (pad_message_list_.size() > FLAGS_control_list_max_size) {
            pad_message_list_.pop_front();
        }
    }
}

const pnc::control::PadMessage ControlMessageManager::GetLatestPadMessage() {
    std::lock_guard<std::mutex> lock(pad_message_mutex_);
    if (!pad_message_list_.empty()) {
        return pad_message_list_.back();
    } else {
        pnc::control::PadMessage pad_message;
        pad_message.mutable_header()->set_module_name("control");
        pad_message.mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
        pad_message.mutable_header()->set_sequence_num(1);
        pad_message.set_action(pnc::control::STOP);
        return pad_message;
    }
}

void ControlMessageManager::SaveControlMessage(const ControlCommandConstPtr& control_command_ptr) {
    if (control_command_ptr == nullptr) {
        LOG_ERROR("Control sanity check failed!");
        return;
    }

    auto control_msg = std::make_shared<pnc::control::ControlCommand>();
    control_msg->CopyFrom(*control_command_ptr);

    if (FLAGS_enable_logsim_mode) {
        control_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
    }

    if (control_msg->has_header()) {
        double conrtol_time_delay = TimeUtility::GetCurrentTimesecond() - control_msg->header().timestamp_sec();
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("status_time_delay: %.6f, status_seq: %d", conrtol_time_delay,
                     control_msg->header().sequence_num());
        }
        if (conrtol_time_delay < -0.001) {
            control_msg->mutable_header()->set_timestamp_sec(TimeUtility::GetCurrentTimesecond());
            LOG_ERROR("status_time_behind: %.6f, status_seq: %d", conrtol_time_delay,
                      control_msg->header().sequence_num());
        }
    } else {
        LOG_ERROR("no timestamp in control msg");
        return;
    }
    // add by chi. check whether same msg
    if (!control_command_list_.empty()) {
        if (std::abs(control_msg->header().timestamp_sec() - control_command_list_.back()->header().timestamp_sec()) <
                1e-2 &&
            control_msg->header().sequence_num() == control_command_list_.back()->header().sequence_num()) {
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(control_command_mutex_);
        control_command_list_.push_back(control_msg);
        if (control_command_list_.size() > FLAGS_control_list_max_size) {
            control_command_list_.pop_front();
        }
    }

    return;
}

ControlCommandConstPtr ControlMessageManager::GetLatestControlMessage() {
    std::lock_guard<std::mutex> lock(control_command_mutex_);
    if (!control_command_list_.empty()) {
        double time_interval =
            TimeUtility::GetCurrentTimesecond() - control_command_list_.back()->header().timestamp_sec();
        if (time_interval < max_control_interval_sec_) {
            return control_command_list_.back();
        } else {
            LOG_ERROR("control message is lost, time_gap: %.4f,  setting: %.4f", time_interval,
                      max_control_interval_sec_);
            return nullptr;
        }
    } else {
        LOG_WARN("control message list is empty");
        return nullptr;
    }
}

}  // namespace control
}  // namespace jiduauto
