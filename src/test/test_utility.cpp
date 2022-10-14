/*! \file test_utility.cpp
 */
#include <eros/Utility.h>
#include <gtest/gtest.h>

using namespace eros;

TEST(ExecTests, ExecCases) {
    {
        ExecResult res = exec("date", false);
        EXPECT_FALSE(res.AnyError);
        EXPECT_EQ(res.ErrorString, "");
        EXPECT_EQ(res.Result, "");

        res = exec("date", true);
        EXPECT_FALSE(res.AnyError);
        EXPECT_EQ(res.ErrorString, "");
        EXPECT_NE(res.Result, "");
    }
}
TEST(UtilityEquality, TestCases) {
    {
        double a = 0.0;
        double b = 0.0;
        double eps = 0.1;
        EXPECT_TRUE(isEqual(a, b, eps));
    }
    {
        double a = 0.0;
        double b = 10.0;
        double eps = 0.1;
        EXPECT_FALSE(isEqual(a, b, eps));
    }
}
TEST(PrettyFunctions, PrettyCases) {
    {
        eros::file msg;
        EXPECT_GT(pretty(msg).size(), 0);
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}