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

#include "control/src/control_component.h"

#include <string>

#include "control/src/common/control_gflags.h"
#include "control/src/common/control_utility.h"
#include "control/src/common/file_util.h"
#include "control/src/math/control_math.h"
#include "pnc_common/src/common/pnc_gflag.h"
#include "pnc_common/src/common/pnc_logger.h"
#include "pnc_common/src/utility/pnc_utility.h"
#include "pnc_common/src/utility/time_utility.h"
#include "pnc_common/src/vehicle/vehicle_config.h"

namespace jiduauto {
namespace control {

using apollo::os::computing::Node;
using apollo::os::computing::Subscriber;
using jiduauto::pnc::util::PnCUtility;
using jiduauto::pnc::util::TimeUtility;
using pnc::chassis::Chassis;
using pnc::control::ControlCommand;
using pnc::control::ControlDebug;
using pnc::localization::LocalizationEstimate;
using pnc::planning::ADCTrajectory;

ControlComponent::ControlComponent() {}

ControlComponent::~ControlComponent() {
    quit_function_thread_ = true;
    if (proc_thread_ != nullptr && proc_thread_->joinable()) {
        proc_thread_->join();
    }
}

/**
 * @brief CHECK only used in Init function.
 */
bool ControlComponent::Init(const std::string& mode) {
    LOG_INFO("ControlComponent init started.");
    CHECK(_node != nullptr) << "_node is nullptr";

    if (!_node->config().empty()) {
        LOG_INFO("flagfile path: %s", _node->config().c_str());
        google::SetCommandLineOption("flagfile", _node->config().c_str());
    }

    if (!PnCUtility::ReadPnCPath()) {
        LOG_WARN("PnCUtility::ReadPnCPath() failed!! Use default gflags");
    }
    if (!FileUtil::ReadPnCPath()) {
        LOG_WARN("FileUtil::ReadPnCPath() failed!! Set JIDU_PNC_PATH!!");
    }

    std::string file_name = FLAGS_control_conf_path + FLAGS_control_conf_file;
    CHECK(PnCUtility::LoadTextFile(file_name, &control_conf_)) << "LoadTextFile failed: " << file_name;
    file_name = FLAGS_control_conf_path + FLAGS_control_chassis_test_file;
    CHECK(PnCUtility::LoadTextFile(file_name, control_conf_.mutable_chassis_test_controller_conf()))
        << "LoadTextFile failed: " << file_name;

    control_v2_ = std::make_unique<ControlRegulatorV2>(control_conf_);
    CHECK(control_v2_ != nullptr) << "ControlRegulator failed.";
    CHECK(control_v2_->Init()) << "ControlRegulator init failed.";

    // init reader
    // TODO(chi.zhang): Evaluate recall and active reader.
    // writer
    cmd_writer_ = _node->CreatePublisher<ControlCommand>(FLAGS_control_topic_name);
    debug_writer_ = _node->CreatePublisher<ControlDebug>(FLAGS_control_debug_topic_name);

    sub_planning_ = _node->CreateSubscriber<ADCTrajectory>(FLAGS_planning_topic_name);
    sub_chassis_ = _node->CreateSubscriber<Chassis>(FLAGS_chassis_topic_name);
    sub_localization_ = _node->CreateSubscriber<LocalizationEstimate>(FLAGS_localization_topic_name);
    sub_padmsg_ = _node->CreateSubscriber<pnc::control::PadMessage>(FLAGS_padmsg_topic_name);
    sub_control_ = _node->CreateSubscriber<pnc::control::ControlCommand>(FLAGS_control_topic_name);

    CHECK(cmd_writer_ != nullptr);
    CHECK(debug_writer_ != nullptr);
    CHECK(sub_planning_ != nullptr);
    CHECK(sub_chassis_ != nullptr);
    CHECK(sub_localization_ != nullptr);
    CHECK(sub_control_ != nullptr);

    // thread
    proc_thread_ = std::make_unique<std::thread>([this]() {
        if (!this->Process()) {
            LOG_ERROR("ControlComponent Process failed!");
        }
    });
    CHECK(proc_thread_ != nullptr);

    LOG_INFO("ControlComponent init finished.");
    return true;
}

bool ControlComponent::Process() {
    uint64_t period_time_ms = static_cast<uint64_t>(control_conf_.control_period() * 1000);

    uint64_t start_time = 0;
    uint64_t end_time = 0;
    uint64_t delta_time = 0;
    while (!apollo::os::computing::IsShutdown() && !quit_function_thread_) {
        start_time = TimeUtility::GetCurrentTimeMillsecond();
        OnAllReader();
        control_v2_->IterationRoutine();

        SendCmd();

        end_time = TimeUtility::GetCurrentTimeMillsecond();
        if (end_time < start_time) {
            LOG_ERROR("end_time < start time, error");
            continue;
        }
        delta_time = end_time - start_time;
        if (FLAGS_enable_control_info_print) {
            LOG_INFO("[Total] control time: %u ms", delta_time);
        }
        if (delta_time < period_time_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(period_time_ms - delta_time));
        }
    }
    return true;
}

void ControlComponent::SendCmd() {
    auto cmd_msg_v2 = control_v2_->GetControlCommand();
    if (cmd_msg_v2 == nullptr) return;
    cmd_writer_->Publish(cmd_msg_v2);

    auto debug_v2 = control_v2_->GetControlDebug();
    if (debug_v2 == nullptr) return;
    debug_writer_->Publish(debug_v2);

    // IDL related
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis>& msg) { control_v2_->SaveChassisMessage(msg); }

void ControlComponent::OnAllReader() {
    // chassis
    auto chassis_msg = sub_chassis_->GetMessageBuffer()->GetLatestMessage<Chassis>();
    if (chassis_msg != nullptr) {
        control_v2_->SaveChassisMessage(chassis_msg);
    }

    // localization
    auto loc_msg = sub_localization_->GetMessageBuffer()->GetLatestMessage<LocalizationEstimate>();
    if (loc_msg != nullptr) {
        control_v2_->SaveLocalizationMessage(loc_msg);
    }

    // planning
    auto planning_msg = sub_planning_->GetMessageBuffer()->GetLatestMessage<ADCTrajectory>();
    if (planning_msg != nullptr) {
        control_v2_->SavePlanningMessage(planning_msg);
    }

    // pad_msg
    auto pad_msg = sub_padmsg_->GetMessageBuffer()->GetLatestMessage<pnc::control::PadMessage>();
    if (pad_msg != nullptr) {
        control_v2_->SavePadMessage(pad_msg);
    }

    // control
    auto control_msg = sub_control_->GetMessageBuffer()->GetLatestMessage<pnc::control::ControlCommand>();
    if (control_msg != nullptr) {
        control_v2_->SaveControlMessage(control_msg);
    }
}

void ControlComponent::OnLocalization(const std::shared_ptr<LocalizationEstimate>& msg) {
    control_v2_->SaveLocalizationMessage(msg);
}

void ControlComponent::OnPlanning(const std::shared_ptr<ADCTrajectory>& msg) { control_v2_->SavePlanningMessage(msg); }

void ControlComponent::OnPadMsg(const std::shared_ptr<pnc::control::PadMessage>& msg) {
    control_v2_->SavePadMessage(msg);
}

void ControlComponent::OnLastControlMsg(const std::shared_ptr<pnc::control::ControlCommand>& msg) {
    control_v2_->SaveControlMessage(msg);
}

}  // namespace control
}  // namespace jiduauto
