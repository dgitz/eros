/*! \file test_SystemMonitorProcessManager.cpp
 */
#include <eros/Logger.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/BaseProcess.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/ProcessManager.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
double COMMTIMEOUT_S = 2.0;
class BaseProcessTester : public BaseProcess
{
   public:
    BaseProcessTester(eros::Logger* logger, std::string nodeName)
        : BaseProcess(logger, IProcess::ProcessType::GENERIC, "", nodeName, COMMTIMEOUT_S) {
    }
    bool setNodeAlive(double currentTime_s) {
        return BaseProcess::setNodeAlive(currentTime_s);
    }
};
TEST(DefinitionTest, TestProcessDefinitions) {
    for (uint8_t i = 0; i <= (uint8_t)(IProcess::ProcessType::END_OF_LIST); ++i) {
        if ((i == 0) || (i == (uint8_t)(IProcess::ProcessType::END_OF_LIST))) {
            EXPECT_TRUE(IProcess::ProcessTypeString((IProcess::ProcessType)(i)) == "UNKNOWN");
            EXPECT_TRUE(IProcess::ProcessTypeEnum(IProcess::ProcessTypeString(
                            (IProcess::ProcessType)(i))) == IProcess::ProcessType::UNKNOWN);
        }
        else {
            EXPECT_FALSE(IProcess::ProcessTypeString((IProcess::ProcessType)(i)) == "UNKNOWN");
            EXPECT_TRUE(IProcess::ProcessTypeEnum(IProcess::ProcessTypeString(
                            (IProcess::ProcessType)(i))) == (IProcess::ProcessType)(i));
        }
    }
}
TEST(BasicTest, TestBaseProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");

    BaseProcessTester SUT(logger, "BaseProcessTester");
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
    EXPECT_TRUE(SUT.setNodeAlive(currentTime_s));
    EXPECT_TRUE(SUT.update(currentTime_s += 0.1 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());

    EXPECT_EQ(SUT.getLevel(), Level::Type::INFO);
    delete logger;
}

TEST(BasicTest, TestProcessManager_AllEROSProcesses) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    ProcessManager SUT(logger, COMMTIMEOUT_S);
    EXPECT_EQ(SUT.getProcesses().size(), 0);
    logger->log_debug(SUT.pretty());
    double currentTime_s = 0.0;
    double dt = 0.01;

    {  // 1 Process
        eros::heartbeat heartbeat;
        heartbeat.NodeName = "Process1";
        heartbeat.stamp.fromSec(currentTime_s += dt);
        EXPECT_TRUE(SUT.new_heartbeat(heartbeat));
        EXPECT_EQ(SUT.getProcesses().size(), 1);
        logger->log_debug(SUT.pretty());
        for (auto proc : SUT.getProcesses()) { EXPECT_GT(proc.second->getAliveCount(), 0); }

        eros::resource resource;
        resource.Name = heartbeat.NodeName;
        resource.stamp.fromSec(currentTime_s += dt);
        EXPECT_TRUE(SUT.new_resourceUsed(resource));
        EXPECT_EQ(SUT.getProcesses().size(), 1);
        logger->log_debug(SUT.pretty());
        for (auto proc : SUT.getProcesses()) { EXPECT_GT(proc.second->getAliveCount(), 1); }
    }

    {  // 2nd Process
        eros::resource resource;
        resource.Name = "Process2";
        resource.stamp.fromSec(currentTime_s += dt);
        EXPECT_TRUE(SUT.new_resourceUsed(resource));

        EXPECT_EQ(SUT.getProcesses().size(), 2);
        logger->log_debug(SUT.pretty());
        for (auto proc : SUT.getProcesses()) { EXPECT_GT(proc.second->getAliveCount(), 0); }

        eros::heartbeat heartbeat;
        heartbeat.NodeName = resource.Name;
        heartbeat.stamp.fromSec(currentTime_s += dt);
        EXPECT_TRUE(SUT.new_heartbeat(heartbeat));
        EXPECT_EQ(SUT.getProcesses().size(), 2);
        logger->log_debug(SUT.pretty());
        for (auto proc : SUT.getProcesses()) { EXPECT_GT(proc.second->getAliveCount(), 1); }
    }
    EXPECT_TRUE(SUT.update(currentTime_s += 1.1 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());
    for (auto proc : SUT.getProcesses()) {
        EXPECT_TRUE(proc.second->getLevel() > Level::Type::NOTICE);
    }

    // Add a bunch more processes
    for (int i = 3; i < 53; ++i) {
        eros::heartbeat heartbeat;
        heartbeat.NodeName = "Process" + std::to_string(i);
        EXPECT_TRUE(SUT.new_heartbeat(heartbeat));
        EXPECT_EQ(SUT.getProcesses().size(), i);
        logger->log_debug(SUT.pretty("\t", ""));
    }

    logger->log_debug(SUT.pretty("\t", ""));
    for (auto proc : SUT.getProcesses()) {
        EXPECT_GT(proc.second->getAliveCount(), 0);
        EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::EROS);
    }
    delete logger;
}

TEST(BasicTest, TestProcessManager_AllGenericProcesses) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    ProcessManager SUT(logger, COMMTIMEOUT_S);
    EXPECT_EQ(SUT.getProcesses().size(), 0);
    logger->log_debug(SUT.pretty());
    double currentTime_s = 0.0;
    double dt = 0.001;
    {  // 1 Process
        EXPECT_TRUE(SUT.new_nodeAlive("Process1", currentTime_s += dt));
        logger->log_debug(SUT.pretty());
        for (auto proc : SUT.getProcesses()) { EXPECT_GT(proc.second->getAliveCount(), 0); }
    }

    logger->log_debug(SUT.pretty("\t", ""));
    for (auto proc : SUT.getProcesses()) {
        EXPECT_GT(proc.second->getAliveCount(), 0);
        EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::GENERIC);
    }

    // Add a bunch more processes
    for (int i = 2; i < 52; ++i) {
        EXPECT_TRUE(SUT.new_nodeAlive("Process" + std::to_string(i), currentTime_s += dt));
        EXPECT_EQ(SUT.getProcesses().size(), i);
        logger->log_debug(SUT.pretty("\t", ""));
    }

    logger->log_debug(SUT.pretty("\t", ""));
    for (auto proc : SUT.getProcesses()) {
        EXPECT_GT(proc.second->getAliveCount(), 0);
        EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::GENERIC);
    }
    delete logger;
}
TEST(BasicTest, TestProcessManager_GenericToEROSProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    ProcessManager SUT(logger, COMMTIMEOUT_S);
    EXPECT_EQ(SUT.getProcesses().size(), 0);
    logger->log_debug(SUT.pretty());
    double currentTime_s = 0.0;
    double dt = 0.001;
    {  // 1 Process
        EXPECT_TRUE(SUT.new_nodeAlive("Process1", currentTime_s += dt));
        EXPECT_EQ(SUT.getProcesses().size(), 1);
        logger->log_debug(SUT.pretty());
        for (auto proc : SUT.getProcesses()) { EXPECT_GT(proc.second->getAliveCount(), 0); }
    }

    for (auto proc : SUT.getProcesses()) {
        EXPECT_GT(proc.second->getAliveCount(), 0);
        EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::GENERIC);
    }
    {
        // 1 Process
        eros::heartbeat heartbeat;
        heartbeat.NodeName = "Process1";
        EXPECT_TRUE(SUT.new_heartbeat(heartbeat));
        EXPECT_EQ(SUT.getProcesses().size(), 1);
    }
    for (auto proc : SUT.getProcesses()) {
        EXPECT_GT(proc.second->getAliveCount(), 0);
        EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::EROS);
    }
    // Create 10 more generic processes
    for (int i = 2; i < 12; ++i) {
        EXPECT_TRUE(SUT.new_nodeAlive("Process" + std::to_string(i), currentTime_s += dt));
        EXPECT_EQ(SUT.getProcesses().size(), i);
        logger->log_debug(SUT.pretty());
        for (auto proc : SUT.getProcesses()) { EXPECT_GT(proc.second->getAliveCount(), 0); }
    }
    logger->log_debug(SUT.pretty());

    EXPECT_GT(SUT.getProcesses().size(), 9);
    {
        uint16_t index = 0;
        for (auto proc : SUT.getProcesses()) {
            if (index == 0) {
                EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::EROS);
            }
            else {
                EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::GENERIC);
            }
            index++;
        }
    }
    // Convert all processes to EROS
    {
        uint16_t index = 0;
        uint16_t startSize = SUT.getProcesses().size();
        for (auto proc : SUT.getProcesses()) {
            eros::resource res;
            res.Name = proc.second->getNodeName();
            res.PID = index + 1;
            res.stamp.fromSec(currentTime_s += dt);
            EXPECT_TRUE(SUT.new_resourceUsed(res));
            EXPECT_EQ(SUT.getProcesses().size(), startSize);
            index++;
        }
    }
    for (auto proc : SUT.getProcesses()) {
        EXPECT_EQ(proc.second->getProcessType(), IProcess::ProcessType::EROS);
    }
    logger->log_debug(SUT.pretty());

    EXPECT_TRUE(SUT.update(currentTime_s += 1.1 * COMMTIMEOUT_S));
    for (auto proc : SUT.getProcesses()) {
        EXPECT_TRUE(proc.second->getLevel() > Level::Type::NOTICE);
    }

    for (auto proc : SUT.getProcesses()) {
        eros::heartbeat heartbeat;
        heartbeat.NodeName = proc.second->getNodeName();
        heartbeat.stamp.fromSec(currentTime_s += dt);
        EXPECT_TRUE(SUT.new_heartbeat(heartbeat));
    }
    EXPECT_TRUE(SUT.update(dt));
    for (auto proc : SUT.getProcesses()) {
        EXPECT_TRUE(proc.second->getLevel() <= Level::Type::NOTICE);
    }

    if (0) {  // May have to enable
        uint64_t currentAliveCount = SUT.getProcesses().find("Process1")->second->getAliveCount();
        EXPECT_TRUE(SUT.new_nodeAlive("Process1", currentTime_s));
        EXPECT_GT(SUT.getProcesses().find("Process1")->second->getAliveCount(), currentAliveCount);
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
