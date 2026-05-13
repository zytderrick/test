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

#include <pavaro/component/component.h>
#include <pavaro/pavaro.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "control/src/control_regulator_v2.h"
#include "pnc_control_cmd.pb.h"
#include "pnc_debug_control.pb.h"
#include "pnc_localization_estimate.pb.h"
#include "pnc_planning.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

/**
 * @brief Main function of control component.
 * Modify it according to middleware/framework.
 */
class ControlComponent : public apollo::os::computing::Component<> {
public:
    ControlComponent();
    ~ControlComponent() override;

    /**
     * @brief Initialize paras.
     * @return true if succeeded; otherwise false;
     */
    bool Init(const std::string& mode = "") override;

private:
    /**
     * @brief Main function, works every control period.
     * @return true if succeeded; otherwise false;
     */
    bool Process();

    /**
     * @brief Send msg out. send proto&JIDL.
     */
    void SendCmd();

    /**
     * @brief Reader processors--proto related.
     */
    void OnAllReader();
    void OnChassis(const std::shared_ptr<pnc::chassis::Chassis>& msg);

    void OnLocalization(const std::shared_ptr<pnc::localization::LocalizationEstimate>& msg);

    void OnPlanning(const std::shared_ptr<pnc::planning::ADCTrajectory>& msg);

    void OnPadMsg(const std::shared_ptr<pnc::control::PadMessage>& msg);

    void OnLastControlMsg(const std::shared_ptr<pnc::control::ControlCommand>& msg);

private:
    std::atomic<bool> quit_function_thread_{false};

    std::unique_ptr<std::thread> proc_thread_{nullptr};
    std::unique_ptr<ControlRegulatorV2> control_v2_{nullptr};
    pnc::control::ControlConfig control_conf_;

    // writer
    std::shared_ptr<apollo::os::computing::Publisher<pnc::control::ControlCommand>> cmd_writer_{nullptr};
    std::shared_ptr<apollo::os::computing::Publisher<pnc::control::ControlDebug>> debug_writer_{nullptr};

    // reader
    std::shared_ptr<apollo::os::computing::Subscriber<pnc::planning::ADCTrajectory>> sub_planning_{nullptr};
    std::shared_ptr<apollo::os::computing::Subscriber<pnc::chassis::Chassis>> sub_chassis_{nullptr};
    std::shared_ptr<apollo::os::computing::Subscriber<pnc::localization::LocalizationEstimate>> sub_localization_{
        nullptr};
    std::shared_ptr<apollo::os::computing::Subscriber<pnc::control::PadMessage>> sub_padmsg_{nullptr};
    std::shared_ptr<apollo::os::computing::Subscriber<pnc::control::ControlCommand>> sub_control_{nullptr};
};

PAVARO_REGISTER_COMPONENT(ControlComponent);
}  // namespace control
}  // namespace jiduauto
