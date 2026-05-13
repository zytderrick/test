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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <cmath>

#include "control/src/common/control_gflags.h"
#include "control/src/common/file_util.h"
#include "control/src/common/leadlag_controller.h"
#include "control/src/common/mrac_controller.h"
#include "control/src/common/pid_controller.h"
#include "pnc_common/src/common/pnc_gflag.h"
#include "pnc_common/src/math/filters/digital_filter.h"
#include "pnc_common/src/math/filters/hysteresis_filter.h"
#include "pnc_common/src/math/filters/mean_filter.h"

namespace jiduauto {
namespace control {

using pnc::filter::DigitalFilter;
using pnc::filter::HysteresisFilter;
using pnc::filter::MeanFilter;

class ControlFilterAnalysisTest : public ::testing::Test {
public:
    virtual void SetUp() {
        CHECK(FileUtil::ReadPnCPath());
        Init();
    }

    void Init() {
        ts_.clear();
        input_.clear();
        output_.clear();
        const double ts = 0.02;
        for (int i = 0; i < 2000; ++i) {
            ts_.push_back(ts * i);
            input_.push_back(std::sin(ts * i));
            output_.push_back(std::numeric_limits<double>::max());
        }
    }

    bool LogToFile(std::string file_name) {
        std::string name = FLAGS_pnc_data_log_path + "/control/" + file_name + ".csv";
        FILE* log_file = fopen(name.c_str(), "w+");
        if (log_file == nullptr) return false;
        fprintf(log_file, "t, input, output,\n");
        for (std::size_t i = 0; i < ts_.size(); ++i) {
            fprintf(log_file, "%.4f, %.4f, %.4f,\n", ts_[i], input_[i], output_[i]);
        }
        return true;
    }

    std::vector<double> ts_;
    std::vector<double> input_;
    std::vector<double> output_;

private:
};

TEST_F(ControlFilterAnalysisTest, PIDFilterTest) {}

}  // namespace control
}  // namespace jiduauto
