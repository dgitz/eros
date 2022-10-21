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

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    Diagnostic::DiagnosticDefinition diag;
    while (timer <= timeToRun) {
        diag = tester->update(dt, timer);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        timer += dt;
    }
    tester->reset();
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

    logger->log_notice("Check Topic Subscribe List.");
    {
        std::vector<std::string> topic_list = {"/a", "/b", "/c"};
        Diagnostic::DiagnosticDefinition diag = tester->update_topiclist(topic_list);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

        std::vector<std::string> redundant_topic_list = {"/a", "/d", "/e"};
        diag = tester->update_topiclist(redundant_topic_list);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    }
    logger->log_notice("Check Diagnostic Aggregator.");
    {
        Diagnostic::DiagnosticDefinition badDiag("Device1",
                                                 "Node1",
                                                 System::MainSystem::ROVER,
                                                 System::SubSystem::ROBOT_CONTROLLER,
                                                 System::Component::COMMUNICATION,
                                                 Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                                 Diagnostic::Message::DROPPING_PACKETS,
                                                 Level::Type::ERROR,
                                                 "Dropping Packets");
        EXPECT_TRUE(tester->new_external_diagnostic(badDiag));
        logger->log_notice(tester->pretty());
    }
    logger->log_notice("Check Failure Cases");
    {
        Diagnostic::DiagnosticDefinition unknownDiag;
        unknownDiag.type = Diagnostic::DiagnosticType::UNKNOWN;
        EXPECT_FALSE(tester->new_external_diagnostic(unknownDiag));
    }
    tester->cleanup();
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
