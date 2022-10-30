/*! \file test_SystemMonitorProcessManager.cpp
 */
#include <eros/Logger.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Processes/EROSProcess.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Processes/GenericProcess.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
double COMMTIMEOUT_S = 2.0;

TEST(BasicTest, TestEROSProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    EROSProcess SUT(logger, "TestEROSProcess", COMMTIMEOUT_S);

    EXPECT_EQ(SUT.getProcessType(), IProcess::ProcessType::EROS);
    double currentTime_s = 0.0;
    EXPECT_TRUE(SUT.update(currentTime_s));
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);
    EXPECT_TRUE(SUT.update(currentTime_s += COMMTIMEOUT_S / 10.0));
    logger->log_debug(SUT.pretty());

    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);
    EXPECT_TRUE(SUT.update(currentTime_s += 1.1 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.getLevel(), Level::Type::WARN);
    EXPECT_TRUE(SUT.update(currentTime_s += 10.0 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());

    EXPECT_EQ(SUT.getLevel(), Level::Type::ERROR);

    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());
    {
        eros::heartbeat beat;
        beat.stamp.fromSec(currentTime_s);
        beat.NodeName = SUT.getNodeName();
        EXPECT_TRUE(SUT.new_heartbeat(beat));
    }
    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);

    EXPECT_TRUE(SUT.update(currentTime_s += 10.0 * COMMTIMEOUT_S));
    EXPECT_EQ(SUT.getLevel(), Level::Type::ERROR);

    {
        eros::resource res;
        res.PID = 1;
        res.stamp.fromSec(currentTime_s);
        res.Name = SUT.getNodeName();
        EXPECT_TRUE(SUT.new_resourceused(res));
    }
    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);
    EXPECT_EQ(SUT.getRestartCount(), 0);

    {
        eros::resource res;
        res.PID = 2;
        res.stamp.fromSec(currentTime_s);
        res.Name = SUT.getNodeName();
        EXPECT_TRUE(SUT.new_resourceused(res));
    }
    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);
    EXPECT_EQ(SUT.getRestartCount(), 1);

    delete logger;
}

TEST(BasicTest, TestEROSProcess_FailureCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    logger->log_warn("Testing EROS Process Failure Scenario's.");
    EROSProcess SUT(logger, "TestEROSProcess", COMMTIMEOUT_S);
    {
        eros::heartbeat beat;
        EXPECT_FALSE(SUT.new_heartbeat(beat));
    }
    {
        eros::heartbeat beat;
        beat.NodeName = "A Node Name that will never exist.";
        EXPECT_FALSE(SUT.new_heartbeat(beat));
    }
    {
        eros::resource res;
        EXPECT_FALSE(SUT.new_resourceused(res));
    }
    {
        eros::resource res;
        res.Name = "A Node Name that will never exist.";
        EXPECT_FALSE(SUT.new_resourceused(res));
    }

    delete logger;
}
TEST(BasicTest, TestGenericProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    GenericProcess SUT(logger, "TestGenericProcess", COMMTIMEOUT_S);

    EXPECT_EQ(SUT.getProcessType(), IProcess::ProcessType::GENERIC);
    double currentTime_s = 0.0;
    EXPECT_TRUE(SUT.update(currentTime_s));
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);
    EXPECT_TRUE(SUT.update(currentTime_s += COMMTIMEOUT_S / 10.0));
    logger->log_debug(SUT.pretty());

    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);
    EXPECT_TRUE(SUT.update(currentTime_s += 1.1 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.getLevel(), Level::Type::WARN);
    EXPECT_TRUE(SUT.update(currentTime_s += 10.0 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());

    EXPECT_EQ(SUT.getLevel(), Level::Type::ERROR);

    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());

    EXPECT_TRUE(SUT.setNodeAlive(currentTime_s));
    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);

    EXPECT_TRUE(SUT.update(currentTime_s += 10.0 * COMMTIMEOUT_S));
    EXPECT_EQ(SUT.getLevel(), Level::Type::ERROR);

    EXPECT_TRUE(SUT.setNodeAlive(currentTime_s));
    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);

    delete logger;
}
/*
TEST(BasicTest, TestProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");

    {  // eros Node
        double currentTime = 0.0;
        eros::resource resourceInfo;
        resourceInfo.stamp.fromSec(currentTime += COMMTIMEOUT_S / 10.0);
        resourceInfo.PID = 1;
        resourceInfo.CPU_Perc = 2.0;
        resourceInfo.RAM_Perc = 3.0;
        resourceInfo.DISK_Perc = 4.0;
        Process SUT(logger,
                    Process::ProcessType::EROS,
                    "/a/b/Node1",
                    "GenericNode",
                    "MyDevice",
                    COMMTIMEOUT_S,
                    resourceInfo);
        logger->log_debug(SUT.pretty());
        EXPECT_TRUE(SUT.update(currentTime));
        EXPECT_TRUE(SUT.getStatus() == Level::Type::INFO);
        logger->log_debug(SUT.pretty());
        EXPECT_TRUE(SUT.update(currentTime += 1.1 * COMMTIMEOUT_S));
        logger->log_debug(SUT.pretty());
        EXPECT_TRUE(SUT.getStatus() == Level::Type::WARN);
        EXPECT_TRUE(SUT.update(currentTime += 10.0 * COMMTIMEOUT_S));
        EXPECT_TRUE(SUT.getStatus() == Level::Type::ERROR);
        logger->log_debug(SUT.pretty());
        EXPECT_EQ(SUT.getRestartCount(), 0);

        resourceInfo.stamp.fromSec(currentTime += COMMTIMEOUT_S / 10.0);
        EXPECT_TRUE(SUT.new_resourceused(resourceInfo));
        EXPECT_EQ(SUT.getRestartCount(), 0);

        resourceInfo.PID++;
        resourceInfo.stamp.fromSec(currentTime += COMMTIMEOUT_S / 10.0);
        EXPECT_TRUE(SUT.new_resourceused(resourceInfo));
        EXPECT_EQ(SUT.getRestartCount(), 1);
        logger->log_debug(SUT.pretty());
    }
    {  // Non eros Node
        double currentTime = 0.0;
        Process SUT(logger,
                    Process::ProcessType::NON_EROS,
                    "/a/b/Node1",
                    "GenericNode",
                    "MyDevice",
                    COMMTIMEOUT_S);
        logger->log_debug(SUT.pretty());
        EXPECT_TRUE(SUT.update(currentTime));
        EXPECT_TRUE(SUT.getStatus() == Level::Type::INFO);
        logger->log_debug(SUT.pretty());
        EXPECT_TRUE(SUT.update(currentTime += 1.1 * COMMTIMEOUT_S));
        logger->log_debug(SUT.pretty());
        EXPECT_TRUE(SUT.getStatus() == Level::Type::WARN);
        EXPECT_TRUE(SUT.update(currentTime += 10.0 * COMMTIMEOUT_S));
        EXPECT_TRUE(SUT.getStatus() == Level::Type::ERROR);
        logger->log_debug(SUT.pretty());
        EXPECT_EQ(SUT.getRestartCount(), 0);

        EXPECT_TRUE(SUT.setNodeAlive(currentTime += COMMTIMEOUT_S / 10.0));
        EXPECT_TRUE(SUT.update(currentTime += COMMTIMEOUT_S / 10.0));
        EXPECT_TRUE(SUT.getStatus() == Level::Type::INFO);

        logger->log_debug(SUT.pretty());
    }
    delete logger;
}
*/
/*
TEST(BasicTest, TestProcessManager) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    ProcessManager SUT(logger, COMMTIMEOUT_S);
    EXPECT_EQ(SUT.getProcesses().size(), 0);

    {  // 1 Process
        eros::heartbeat heartbeat;
        EXPECT_TRUE(SUT.new_heartbeat(heartbeat));
        EXPECT_EQ(SUT.getProcesses().size(), 1);

        eros::resource resource;
        EXPECT_TRUE(SUT.new_resourceused(resource));
        EXPECT_EQ(SUT.getProcesses().size(), 1);
    }
    logger->log_debug(SUT.pretty());
    delete logger;
}
*/
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
