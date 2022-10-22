/*! \file test_SystemMonitorProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include <eros/SystemMonitor/SystemMonitorProcess.h>
using namespace eros;
class SystemMonitorProcessTester : public SystemMonitorProcess {
    public :
        SystemMonitorProcessTester(){}
        ~SystemMonitorProcessTester(){}
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitorProcess");
    SystemMonitorProcessTester* tester = new SystemMonitorProcessTester;
    tester->initialize("UnitTestSystemMonitorProcess",
                       "UnitTestSystemMonitorProcess",
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

    Diagnostic::DiagnosticDefinition diag = tester->finish_initialization();
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    tester->reset();

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    while (timer <= timeToRun) {
        diag = tester->update(dt, timer);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        timer += dt;
    }

    logger->log_warn("Testing Unsupported Command Message");
    {
        eros::command cmd;
        std::vector<eros::Diagnostic::DiagnosticDefinition> diag_list = tester->new_commandmsg(cmd);
        EXPECT_EQ(diag_list.size(), 0);
    }
    logger->log_warn("Testing Unsupported Program Variables Check");
    {
        std::vector<eros::Diagnostic::DiagnosticDefinition> diag_list =
            tester->check_programvariables();
        EXPECT_EQ(diag_list.size(), 0);
    }
    tester->cleanup();

    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
