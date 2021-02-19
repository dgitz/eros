/*! \file test_MasterNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../MasterNodeProcess.h"

class MasterNodeProcessTester : public MasterNodeProcess
{
   public:
    MasterNodeProcess Tester() {
    }
    ~MasterNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestMasterNodeProcess");
    MasterNodeProcessTester* tester = new MasterNodeProcessTester;
    tester->initialize("UnitTestMasterNodeProcess",
                       "UnitTestMasterNodeProcess",
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

    EXPECT_TRUE(tester->read_device_architecture() != Architecture::Type::UNKNOWN);
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
