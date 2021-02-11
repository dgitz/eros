/*! \file test_diagnostics.cpp
 */
#include <eros/Diagnostic.h>
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <stdio.h>
TEST(BasicTest, TestDefintions) {
    // Test Type: DiagnosticType
    for (uint8_t i = 1; i < (uint8_t)(Diagnostic::DiagnosticType::END_OF_LIST); ++i) {
        EXPECT_FALSE(Diagnostic::DiagnosticTypeString((Diagnostic::DiagnosticType)(i)) ==
                     "UNKNOWN");
    }
    // Test Type: Message
    for (uint8_t i = 1; i < (uint8_t)(Diagnostic::Message::END_OF_LIST); ++i) {
        EXPECT_FALSE(Diagnostic::DiagnosticMessageString((Diagnostic::Message)(i)) == "UNKNOWN");
    }
}
TEST(BasicTest, DiagnosticHelper) {
    Diagnostic diag_helper;
    diag_helper.initialize("UnitTestDevice",
                           "UnitTestNode-Diagnostics",
                           System::MainSystem::SIMROVER,
                           System::SubSystem::ENTIRE_SYSTEM,
                           System::Component::ENTIRE_SUBSYSTEM);
    std::unique_ptr<Logger> logger(new Logger(
        "DEBUG", "/home/robot/var/log/output/", diag_helper.get_root_diagnostic().node_name));
    {
        std::vector<Diagnostic::DiagnosticType> diag_types;
        diag_types.push_back(Diagnostic::DiagnosticType::COMMUNICATIONS);
        diag_types.push_back(Diagnostic::DiagnosticType::DATA_STORAGE);
        diag_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
        EXPECT_TRUE(diag_helper.enable_diagnostics(diag_types));
        printf("Diagnostics:\n");
        printf("%s\n", diag_helper.pretty().c_str());
    }

    {
        Diagnostic::DiagnosticDefinition root_diag = diag_helper.get_root_diagnostic();

        Diagnostic::DiagnosticDefinition diag =
            diag_helper.update_diagnostic(root_diag.device_name,
                                          Diagnostic::DiagnosticType::SENSORS,
                                          Level::Type::INFO,
                                          Diagnostic::Message::NOERROR,
                                          "No Error");
        printf("Diag: %s\n", Diagnostic::pretty("\t", diag).c_str());
        EXPECT_TRUE(logger->log_diagnostic(diag) == Logger::LoggerStatus::LOG_WRITTEN);
        EXPECT_TRUE(diag.level > Level::Type::WARN);
    }
    {
        Diagnostic::DiagnosticDefinition diag =
            diag_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                          Level::Type::INFO,
                                          Diagnostic::Message::NOERROR,
                                          "Running.");
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    }
    {
        Diagnostic::DiagnosticDefinition diag =
            diag_helper.update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::INFO,
                                          Diagnostic::Message::NOERROR,
                                          "Running.");
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    }
    {
        Diagnostic::DiagnosticDefinition diag =
            diag_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                          Level::Type::INFO,
                                          Diagnostic::Message::NOERROR,
                                          "Running.");
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
    }
    {
        std::vector<Diagnostic::DiagnosticDefinition> diagnostics = diag_helper.get_diagnostics();

        for (std::size_t i = 0; i < diagnostics.size(); ++i) {
            printf("Diag: %s\n", Diagnostic::pretty("\t", diagnostics.at(i)).c_str());
            logger->log_diagnostic(diagnostics.at(i));
            EXPECT_TRUE(diagnostics.at(i).level <= Level::Type::NOTICE);
        }
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}