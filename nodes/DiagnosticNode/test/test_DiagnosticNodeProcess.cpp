/*! \file test_DiagnosticNodeProcess.cpp
 */
#include <eros/DiagnosticNode/DiagnosticNodeProcess.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
using namespace eros_nodes;
class DiagnosticNodeProcessTester : public DiagnosticNodeProcess
{
   public:
    DiagnosticNodeProcessTester() {
    }
    ~DiagnosticNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestDiagnosticNodeProcess");
    DiagnosticNodeProcessTester* tester = new DiagnosticNodeProcessTester;
    tester->initialize("UnitTestDiagnosticNodeProcess",
                       "UnitTestDiagnosticNodeProcess",
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
    tester->finish_initialization();
    ;

    {
        Diagnostic::DiagnosticDefinition worst_diag =
            tester->get_worst_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL);
        logger->log_diagnostic(worst_diag);
    }
    EXPECT_TRUE(tester->get_worst_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
                Level::Type::INFO);
    {
        Diagnostic::DiagnosticDefinition diag("Device1",
                                              "Node1",
                                              System::MainSystem::ROVER,
                                              System::SubSystem::ROBOT_CONTROLLER,
                                              System::Component::COMMUNICATION,
                                              Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                              Diagnostic::Message::INITIALIZING,
                                              Level::Type::WARN,
                                              "Msg1");
        EXPECT_TRUE(tester->new_external_diagnostic(diag));
    }

    EXPECT_TRUE(tester->get_worst_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
                Level::Type::WARN);
    {
        Diagnostic::DiagnosticDefinition diag("Device1",
                                              "Node1",
                                              System::MainSystem::ROVER,
                                              System::SubSystem::ROBOT_CONTROLLER,
                                              System::Component::DIAGNOSTIC,
                                              Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                              Diagnostic::Message::INITIALIZING,
                                              Level::Type::NOTICE,
                                              "Msg1");
        EXPECT_TRUE(tester->new_external_diagnostic(diag));
    }
    EXPECT_TRUE(tester->get_worst_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
                Level::Type::WARN);
    {
        Diagnostic::DiagnosticDefinition diag("Device1",
                                              "Node1",
                                              System::MainSystem::ROVER,
                                              System::SubSystem::ROBOT_CONTROLLER,
                                              System::Component::COMMUNICATION,
                                              Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                              Diagnostic::Message::INITIALIZING,
                                              Level::Type::NOTICE,
                                              "Msg1");
        EXPECT_TRUE(tester->new_external_diagnostic(diag));
    }
    EXPECT_TRUE(tester->get_worst_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL).level ==
                Level::Type::NOTICE);
    printf("%s\n", tester->pretty().c_str());
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
