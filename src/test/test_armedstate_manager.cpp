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
    std::vector<std::string> ready_to_arm_list;
    ready_to_arm_list.push_back("Signal1");
    Logger* logger = new Logger("INFO", node_name);
    ArmedStateManager SUT(device_name,
                          node_name,
                          System::MainSystem::SIMROVER,
                          System::SubSystem::ROBOT_CONTROLLER,
                          ready_to_arm_list);
    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED_CANNOTARM);
    auto current_diag = SUT.get_current_diagnostic();

    double current_time = 0.0;
    double delta_time = 0.1;
    EXPECT_EQ(current_diag.level, Level::Type::WARN);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::INITIALIZING);
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_TRUE(SUT.new_ready_to_arm_msg("Signal1", true));
    current_diag = SUT.update(current_time += delta_time);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    eros::command cmd;
    cmd.Command = (uint16_t)Command::Type::ARM;
    EXPECT_TRUE(SUT.new_command(cmd));

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMING);

    current_diag = SUT.update(current_time += ArmedStateManager::ARMING_TIME_SEC + delta_time);
    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::ARMED);

    cmd.Command = (uint16_t)Command::Type::DISARM;
    EXPECT_TRUE(SUT.new_command(cmd));

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMING);

    current_diag = SUT.update(current_time += ArmedStateManager::DISARMING_TIME_SEC + delta_time);
    EXPECT_EQ(current_diag.level, Level::Type::INFO);
    EXPECT_EQ(current_diag.message, eros_diagnostic::Message::NOERROR);

    EXPECT_EQ(SUT.get_armed_state().state, ArmDisarm::Type::DISARMED);

    logger->log_info(SUT.pretty());

    delete logger;
}
// Test Case: Multiple Ready To Arm Signals
// Test Case: Multiple Ready To Arm Signal True/False
// Test Case: Ready To Arm Signal Timeout
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
