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
 * @brief copied from pnc_common/unit_test/test_src/math/filters
 * To Verfity 'nan' issues.
 */
#include "pnc_common/src/math/filters/digital_filter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pnc_common/src/utility/random.h"

namespace jiduauto {
namespace control {
using pnc::filter::DigitalFilter;

TEST(DigitalFilter, LpfCoefficients) {
    double ts = 0.01;
    double cutoff_freq = 20;
    std::vector<double> den;
    std::vector<double> num;
    pnc::filter::LpfCoefficients(ts, cutoff_freq, &den, &num);
    EXPECT_EQ(den.size(), 3);
    EXPECT_EQ(num.size(), 3);
    EXPECT_NEAR(num[0], 0.1729, 0.01);
    EXPECT_NEAR(num[1], 0.3458, 0.01);
    EXPECT_NEAR(num[2], 0.1729, 0.01);
    EXPECT_NEAR(den[0], 1.0, 0.01);
    EXPECT_NEAR(den[2], 0.2217, 0.01);
}

TEST(DigitalFilter, LpFirstOrderCoefficients) {
    double ts = 0.01;
    double settling_time = 0.005;
    double dead_time = 0.04;
    std::vector<double> den;
    std::vector<double> num;
    pnc::filter::LpFirstOrderCoefficients(ts, settling_time, dead_time, &den, &num);
    EXPECT_EQ(den.size(), 2);
    EXPECT_EQ(num.size(), 5);
    EXPECT_NEAR(den[1], -0.13533, 0.01);
    EXPECT_DOUBLE_EQ(num[0], 0.0);
    EXPECT_DOUBLE_EQ(num[1], 0.0);
    EXPECT_NEAR(num[4], 1 - 0.13533, 0.01);
    dead_time = 0.0;
    pnc::filter::LpFirstOrderCoefficients(ts, settling_time, dead_time, &den, &num);
    EXPECT_EQ(den.size(), 2);
    EXPECT_EQ(num.size(), 1);
    EXPECT_NEAR(den[1], -0.13533, 0.01);
    EXPECT_NEAR(num[0], 1 - 0.13533, 0.01);
}

TEST(DigitalFilter, Filter) {
    DigitalFilter digital_filter;
    std::vector<double> num = {1.0, 2.0, 3.0};
    std::vector<double> den = {4.0, 5.0, 6.0};
    digital_filter.SetDenominator(den);
    digital_filter.SetNumerator(num);
    std::vector<double> den_result = digital_filter.GetDenominator();
    std::vector<double> num_result = digital_filter.GetNumerator();
    EXPECT_EQ(num_result.size(), num.size());
    EXPECT_EQ(den_result.size(), den.size());
    for (std::size_t i = 0; i < num.size(); ++i) {
        EXPECT_DOUBLE_EQ(num_result[i], num[i]);
    }
    for (std::size_t i = 0; i < den.size(); ++i) {
        EXPECT_DOUBLE_EQ(den_result[i], den[i]);
    }
    digital_filter.SetCoefficients(den, num);
    den_result.clear();
    den_result = digital_filter.GetDenominator();
    num_result.clear();
    num_result = digital_filter.GetNumerator();
    EXPECT_EQ(num_result.size(), num.size());
    EXPECT_EQ(den_result.size(), den.size());
    for (std::size_t i = 0; i < num.size(); ++i) {
        EXPECT_DOUBLE_EQ(num_result[i], num[i]);
    }
    for (std::size_t i = 0; i < den.size(); ++i) {
        EXPECT_DOUBLE_EQ(den_result[i], den[i]);
    }

    double dead_zone = 1.0;
    digital_filter.SetDeadZone(dead_zone);
    EXPECT_FLOAT_EQ(digital_filter.GetDeadZone(), dead_zone);
}

TEST(DigitalFilter, FilterOff) {
    std::vector<double> num = {0.0, 0.0, 0.0};
    std::vector<double> den = {1.0, 0.0, 0.0};
    DigitalFilter digital_filter(den, num);
    unsigned int seed = 0;

    const std::vector<double> step_input(100, 1.0);
    std::vector<double> rand_input(100, 1.0);
    for (std::size_t i = 0; i < rand_input.size(); ++i) {
        rand_input[i] = rand_r(&seed);
    }
    // Check setp input
    for (std::size_t i = 0; i < step_input.size(); ++i) {
        EXPECT_FLOAT_EQ(digital_filter.Filter(step_input[i]), 0.0);
    }
    // Check random input
    for (std::size_t i = 0; i < rand_input.size(); ++i) {
        EXPECT_FLOAT_EQ(digital_filter.Filter(rand_input[i]), 0.0);
    }
}

TEST(DigitalFilter, MovingAverage) {
    std::vector<double> num = {0.25, 0.25, 0.25, 0.25};
    std::vector<double> den = {1.0, 0.0, 0.0};
    DigitalFilter digital_filter;
    digital_filter.SetNumerator(num);
    digital_filter.SetDenominator(den);

    const std::vector<double> step_input(100, 1.0);
    // Check step input, transients.
    for (std::size_t i = 0; i < 4; ++i) {
        double expected_filter_out = (i + 1) * 0.25;
        EXPECT_FLOAT_EQ(digital_filter.Filter(step_input[i]), expected_filter_out);
    }
    // Check step input, steady state
    for (std::size_t i = 4; i < step_input.size(); ++i) {
        EXPECT_FLOAT_EQ(digital_filter.Filter(step_input[i]), 1.0);
    }
}

TEST(DigitalFilter, RandomTest) {
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    pnc::filter::LpfCoefficients(0.02, 10, &den, &num);

    DigitalFilter digital_filter;
    digital_filter.SetCoefficients(den, num);

    double output = 0.0;
    // Check step input, transients.
    for (int i = 0; i < 1000; ++i) {
        output = pnc::util::RandomDouble(-10.0, 10.0);
        EXPECT_TRUE(!std::isnan(output));
        output = digital_filter.Filter(output);
        EXPECT_TRUE(!std::isnan(output));
    }
}

TEST(DigitalFilter, RandomTestReset) {
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    pnc::filter::LpfCoefficients(0.02, 10, &den, &num);

    DigitalFilter digital_filter;
    digital_filter.SetCoefficients(den, num);

    double output = 0.0;
    // Check step input, transients.
    for (int i = 0; i < 1000; ++i) {
        output = pnc::util::RandomDouble(-10.0, 10.0);
        EXPECT_TRUE(!std::isnan(output));
        output = digital_filter.Filter(output);
        EXPECT_TRUE(!std::isnan(output));
        digital_filter.Reset();
    }
}

}  // namespace control
}  // namespace jiduauto
