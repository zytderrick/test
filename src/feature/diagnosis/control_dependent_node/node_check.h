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

/**
 * @file
 * @brief Defines the NodeCheck base class.
 */
#pragma once

#include <queue>
#include <string>
#include <vector>

#include "control/src/common/control_message_stream.h"
#include "pnc_control_config.pb.h"
#include "pnc_debug_control.pb.h"

/**
 * @namespace jiduauto::control
 */
namespace jiduauto {
namespace control {

/**
 * @class NodeCheck
 * @brief base class to check all Control Dependent Nodes .
 */
class NodeCheck {
public:
    NodeCheck() = default;
    virtual ~NodeCheck() = default;

    virtual bool Init(const pnc::control::ControlConfig& control_conf) = 0;

    virtual bool GeneralCheck(const InputStream& input_stream,
                              pnc::control::DiagnosisDebugV2* const diag_debug_ptr) = 0;

    virtual bool SpecificCheck(const ControlInputStream& control_input_stream,
                               pnc::control::DiagnosisDebugV2* const diag_debug_ptr) = 0;

    /**
     * @brief reset NodeCheck
     */
    virtual void Reset() = 0;

    /**
     * @brief NodeCheck name
     * @return string NodeCheck name in string
     */
    virtual std::string Name() const = 0;

    template <typename T>
    void ClearQueue(std::queue<T>* const queue_ptr) {
        while (!queue_ptr->empty()) {
            queue_ptr->pop();
        }
    }

    void SetCodeBitHigh(int* const check_code, const int& bit) { (*check_code) |= (0x00000001 << (bit - 1)); }

    void SetCodeBitLow(int* const check_code, const int& bit) { (*check_code) &= ~(0x00000001 << (bit - 1)); }
};

}  // namespace control
}  // namespace jiduauto
