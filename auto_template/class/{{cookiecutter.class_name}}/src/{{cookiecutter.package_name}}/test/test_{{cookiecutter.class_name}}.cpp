/*! \file test_{{cookiecutter.class_name}}.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>
#include <{{cookiecutter.package_name}}/{{cookiecutter.class_name}}.h>
using namespace eros;
TEST(BasicTest, TestDefintions) {
    // Test Type: Enum1
    for (uint8_t i = 1; i < (uint8_t)({{cookiecutter.class_name}}::Enum1::END_OF_LIST); ++i) {
        EXPECT_FALSE({{cookiecutter.class_name}}::Enum1String(({{cookiecutter.class_name}}::Enum1)(i)) ==
                     "UNKNOWN");
    }
}
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "{{cookiecutter.class_name}}");
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}