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
#include "pnc_common/src/math/filters/mean_filter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pnc_common/src/utility/random.h"

namespace jiduauto {
namespace control {
using pnc::filter::MeanFilter;

TEST(MeanFilterTest, WindowSizeOne) {
    MeanFilter mean_filter(1);

    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 4.0);
}

TEST(MeanFilterTest, WindowSizeTwo) {
    MeanFilter mean_filter(2);

    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 4.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 2.5);
}

TEST(MeanFilterTest, OnePositiveNonZero) {
    MeanFilter mean_filter(5);

    EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 2.0);
}

TEST(MeanFilterTest, OneNegativeNonZero) {
    MeanFilter mean_filter(5);

    EXPECT_DOUBLE_EQ(mean_filter.Update(-2.0), -2.0);
}

TEST(MeanFilterTest, TwoPositiveNonZeros) {
    MeanFilter mean_filter(5);

    mean_filter.Update(3.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 3.5);
}

TEST(MeanFilterTest, TwoNegativeNonZeros) {
    MeanFilter mean_filter(5);

    mean_filter.Update(-3.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(-4.0), -3.5);
}

TEST(MeanFilterTest, OnePositiveOneNegative) {
    MeanFilter mean_filter(5);

    mean_filter.Update(-3.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 0.5);
}

TEST(MeanFilterTest, NormalThree) {
    MeanFilter mean_filter(5);

    mean_filter.Update(-3.0);
    mean_filter.Update(4.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 3.0);
}

TEST(MeanFilterTest, NormalFullFiveExact) {
    MeanFilter mean_filter(5);

    mean_filter.Update(-1.0);
    mean_filter.Update(0.0);
    mean_filter.Update(1.0);
    mean_filter.Update(2.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 1.0);
}

TEST(MeanFilterTest, NormalFullFiveOver) {
    MeanFilter mean_filter(5);

    mean_filter.Update(-1.0);
    mean_filter.Update(0.0);
    mean_filter.Update(1.0);
    mean_filter.Update(2.0);
    mean_filter.Update(3.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 2.0);
}

TEST(MeanFilterTest, SameNumber) {
    MeanFilter mean_filter(5);

    for (int i = 0; i < 10; ++i) {
        EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 1.0);
    }
}

TEST(MeanFilterTest, LargeNumber) {
    MeanFilter mean_filter(10);
    double val{0.0};
    for (int i = 0; i < 100; ++i) {
        double input = static_cast<double>(i);
        val = mean_filter.Update(input);
    }
    EXPECT_DOUBLE_EQ(val, 94.5);
}

TEST(MeanFilterTest, AlmostScale) {
    MeanFilter mean_filter(3);

    EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 1.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 1.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(9.0), 2.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(8.0), 8.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(7.0), 8.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(6.0), 7.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 6.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 5.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 4.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 3.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 2.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 2.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 3.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 4.0);
}

TEST(MeanFilterTest, ToyExample) {
    MeanFilter mean_filter(4);

    EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 5.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 4.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(8.0), 5.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(9.0), 6.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(7.0), 7.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 7.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 4.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 3.0);
}

TEST(MeanFilterTest, GoodMinRemoval) {
    MeanFilter mean_filter(2);

    EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 1.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(9.0), 5.0);
    EXPECT_DOUBLE_EQ(mean_filter.Update(8.0), 8.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(7.0), 7.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(6.0), 6.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 5.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 4.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 3.5);
    EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 2.5);
}

TEST(MeanFilterTest, RandomTest) {
    MeanFilter mean_filter;
    mean_filter.SetWindowSize(10);
    double output = 0.0;
    // Check step input, transients.
    for (int i = 0; i < 1000; ++i) {
        output = pnc::util::RandomDouble(-10.0, 10.0);
        EXPECT_TRUE(!std::isnan(output));
        output = mean_filter.Update(output);
        EXPECT_TRUE(!std::isnan(output));
    }
}

TEST(MeanFilterTest, RandomTestReset) {
    MeanFilter mean_filter;
    mean_filter.SetWindowSize(10);
    double output = 0.0;
    // Check step input, transients.
    for (int i = 0; i < 1000; ++i) {
        output = pnc::util::RandomDouble(-10.0, 10.0);
        EXPECT_TRUE(!std::isnan(output));
        output = mean_filter.Update(output);
        EXPECT_TRUE(!std::isnan(output));
        mean_filter.Reset();
    }
}

}  // namespace control
}  // namespace jiduauto
