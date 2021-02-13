/*! \file test_systemmonitor_process.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../SystemMonitorProcess.h"

class SystemMonitorProcessTester : public SystemMonitorProcess
{
   public:
    SystemMonitorProcessTester() {
    }
    ~SystemMonitorProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger =
        new Logger("DEBUG", "/home/robot/var/log/output", "UnitTestSystemMonitorProcess");
    SystemMonitorProcessTester* tester = new SystemMonitorProcessTester;
    tester->initialize("UnitTestSystemMonitorProcess",
                       "UnitTestSystemMonitorProcess",
                       "MyHost",
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}