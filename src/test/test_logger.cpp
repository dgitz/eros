/*! \file test_diagnostics.cpp
 */
#include <eros/Diagnostic.h>
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <stdio.h>
TEST(BasicTest, TestOperation) {
    Diagnostic diag_helper;
    diag_helper.initialize("UnitTestDevice",
                           "UnitTestNode-Logger",
                           System::MainSystem::SIMROVER,
                           System::SubSystem::ENTIRE_SYSTEM,
                           System::Component::ENTIRE_SUBSYSTEM);
    Logger* logger = new Logger("INFO", diag_helper.get_root_diagnostic().node_name);
    EXPECT_TRUE(logger->enable_ROS_logger() == true);
    EXPECT_TRUE(logger->log_debug("A String that should debug") ==
                Logger::LoggerStatus::LOG_SUPPRESSED);
    EXPECT_TRUE(logger->log_info("A String that should info") == Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_notice("A String that should notice") ==
                Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_warn("A String that should warn") == Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_error("A String that should error") ==
                Logger::LoggerStatus::LOG_WRITTEN);
    EXPECT_TRUE(logger->log_fatal("A String that should fatal") ==
                Logger::LoggerStatus::LOG_WRITTEN);
    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}