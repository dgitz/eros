/*! \file test_SafetyNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../SafetyNodeProcess.h"

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
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    EXPECT_TRUE(tester->get_armed_state().state == ArmDisarm::Type::DISARMED_CANNOTARM);

    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
