/*! \file test_armedstate_manager.cpp
 */
#include <eros/ArmedStateManager.h>
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestOperation_HappyFlow) {
    std::string node_name = "UnitTestArmedStateManager";
    std::string device_name = "UnitTest";
    Logger* logger = new Logger("INFO", node_name);
    ArmedStateManager SUT(
        device_name, node_name, System::MainSystem::SIMROVER, System::SubSystem::ROBOT_CONTROLLER);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto current_diag = SUT.get_current_diagnostic();

    double current_time = 0.0;
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::INITIALIZING);
    current_diag = SUT.update(0.0, 0.0);

    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMING);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMED);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMING);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);
    logger->log_info(SUT.pretty());

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
