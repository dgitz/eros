/*! \file test_SafetyNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "SafetyNodeProcess.h"
using namespace eros;
using namespace eros_nodes;
class SafetyNodeProcessTester : public SafetyNodeProcess
{
   public:
    SafetyNodeProcessTester() {
    }
    ~SafetyNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSafetyNodeProcess");
    SafetyNodeProcessTester* tester = new SafetyNodeProcessTester;
    tester->initialize("UnitTestSafetyNodeProcess",
                       "UnitTestSafetyNodeProcess",
                       "MyHost",
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::REMOTE_CONTROL);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED_CANNOTARM);

    {  // Create Ready To Arm Monitors
        std::vector<std::string> topics;
        topics.push_back("Topic1");
        topics.push_back("Topic2");
        topics.push_back("Topic3");

        std::vector<ArmDisarmMonitor::Type> types;
        types.push_back(ArmDisarmMonitor::Type::DEFAULT);
        types.push_back(ArmDisarmMonitor::Type::SIMPLE);
        types.push_back(ArmDisarmMonitor::Type::DEFAULT);
        EXPECT_TRUE(tester->initialize_readytoarm_monitors(topics, types) == true);
    }

    EXPECT_TRUE(tester->get_armed_state().state ==
                ArmDisarm::Type::DISARMED_CANNOTARM);  // No Armed Monitors Updated Yet.
    tester->update(0.1, 0.1);
    {
        std::vector<std::string> cannotarm_reasons = tester->get_cannotarm_reasons();
        EXPECT_TRUE(cannotarm_reasons.size() > 0);
        for (auto reason : cannotarm_reasons) { logger->log_warn(reason); }
    }

    {
        EXPECT_FALSE(tester->new_message_readytoarm("Topic1", true));
        eros::ready_to_arm arm;
        arm.ready_to_arm = true;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic1", arm));
    }
    { EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true)); }
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = true;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic3", arm));
    }
    tester->update(0.1, 0.2);
    {
        std::vector<std::string> cannotarm_reasons = tester->get_cannotarm_reasons();
        EXPECT_TRUE(cannotarm_reasons.size() == 0);
    }
    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED);
    EXPECT_TRUE(tester->new_message_readytoarm("Topic2", false));
    tester->update(0.1, 0.3);
    {
        std::vector<std::string> cannotarm_reasons = tester->get_cannotarm_reasons();
        EXPECT_TRUE(cannotarm_reasons.size() == 1);
        for (auto reason : cannotarm_reasons) { logger->log_warn(reason); }
    }
    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED_CANNOTARM);

    EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true));
    tester->update(0.1, 0.4);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 0);
    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED);
    {  // Arm System
        eros::command command;
        command.Command = (uint16_t)Command::Type::ARM;
        std::vector<Diagnostic::DiagnosticDefinition> diag_list = tester->new_commandmsg(command);
        EXPECT_TRUE(diag_list.size() > 0);
        for (auto diag : diag_list) {
            logger->log_diagnostic(diag);
            EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        }
        EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::ARMING);
        tester->update(0.1, 0.5);
        EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::ARMED);
    }
    // Change a Topic to Not Read to Arm
    EXPECT_TRUE(tester->new_message_readytoarm("Topic2", false));
    tester->update(0.1, 0.6);
    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 1);
    // Change another topic to not ready to arm
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = false;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic3", arm));
    }
    tester->update(0.1, 0.7);
    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 2);
    // Change both back to ready to arm, state should be disarmed
    {
        eros::ready_to_arm arm;
        arm.ready_to_arm = true;
        EXPECT_TRUE(tester->new_message_readytoarm("Topic3", arm));
    }
    tester->update(0.1, 0.8);
    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED_CANNOTARM);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 1);
    EXPECT_TRUE(tester->new_message_readytoarm("Topic2", true));
    tester->update(0.1, 0.9);
    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED);
    EXPECT_TRUE(tester->get_cannotarm_reasons().size() == 0);
    // Arm Again
    {  // Arm System
        eros::command command;
        command.Command = (uint16_t)Command::Type::ARM;
        std::vector<Diagnostic::DiagnosticDefinition> diag_list = tester->new_commandmsg(command);
        EXPECT_TRUE(diag_list.size() > 0);
        for (auto diag : diag_list) {
            logger->log_diagnostic(diag);
            EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        }
        EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::ARMING);
        tester->update(0.1, 1.0);
        EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::ARMED);
    }
    // Disarm Normally via command

    {  // Disarm System
        eros::command command;
        command.Command = (uint16_t)Command::Type::DISARM;
        std::vector<Diagnostic::DiagnosticDefinition> diag_list = tester->new_commandmsg(command);
        EXPECT_TRUE(diag_list.size() > 0);
        for (auto diag : diag_list) {
            logger->log_diagnostic(diag);
            EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        }
        EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMING);
        tester->update(0.1, 1.1);
        EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED);
    }

    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
