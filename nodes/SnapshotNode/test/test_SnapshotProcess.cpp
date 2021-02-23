/*! \file test_SnapshotProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../SnapshotProcess.h"

class SnapshotProcessTester : public SnapshotProcess
{
   public:
    SnapshotProcessTester() {
    }
    ~SnapshotProcessTester() {
    }
};
TEST(BasicTest, TestOperation_Master) {
    Logger* logger = new Logger("DEBUG", "UnitTestSnapshotProcess");
    SnapshotProcessTester* tester = new SnapshotProcessTester;
    Diagnostic::DiagnosticDefinition diag;
    tester->initialize("UnitTestSnapshotProcess",
                       "UnitTestSnapshotProcess",
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
    tester->set_mode(SnapshotProcess::Mode::MASTER);
    tester->set_architecture(Architecture::Type::X86_64);

    diag = tester->load_config("/home/robot/config/Some file that doesn't exist.xml");
    EXPECT_TRUE(diag.level >= Level::Type::ERROR);

    diag = tester->load_config("/home/robot/config/SnapshotConfig.xml");
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    logger->log_notice("\n" + tester->pretty());

    diag = tester->finish_initialization();

    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    std::vector<Diagnostic::DiagnosticDefinition> diag_list = tester->createnew_snapshot();
    EXPECT_TRUE(diag_list.size() > 0);
    for (std::size_t i = 0; i < diag_list.size(); ++i) {
        logger->log_diagnostic(diag_list.at(i));
        EXPECT_TRUE(diag_list.at(i).level <= Level::Type::NOTICE);
    }
    EXPECT_TRUE(tester->get_devicesnapshot_state() == SnapshotProcess::SnapshotState::NOTRUNNING);
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
