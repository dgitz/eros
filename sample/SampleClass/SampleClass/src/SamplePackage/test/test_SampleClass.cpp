/*! \file test_SampleClass.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>
#include <SamplePackage/SampleClass.h>
using namespace eros;
TEST(BasicTest, TestDefintions) {
    // Test Type: Enum1
    for (uint8_t i = 1; i < (uint8_t)(SampleClass::Enum1::END_OF_LIST); ++i) {
        EXPECT_FALSE(SampleClass::Enum1String((SampleClass::Enum1)(i)) ==
                     "UNKNOWN");
    }
}
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "SampleClass");
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}