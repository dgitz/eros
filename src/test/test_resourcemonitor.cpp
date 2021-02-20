/*! \file test_resourcemonitor.cpp
 */
#include <eros/Logger.h>
#include <eros/ResourceMonitor.h>
#include <gtest/gtest.h>

#include "../../nodes/MasterNode/MasterNodeProcess.h"
TEST(BasicTest, TestOperation_Node) {
    const double TIME_TO_RUN = 30000.0;
    Logger* logger = new Logger("INFO", "UnitTestResourceMonitor");

    MasterNodeProcess* master_node_process = new MasterNodeProcess;
    master_node_process->initialize("UnitTestResourceMonitor",
                                    "UnitTestResouceMonitor",
                                    "MyHost",
                                    System::MainSystem::SIMROVER,
                                    System::SubSystem::ENTIRE_SYSTEM,
                                    System::Component::ENTIRE_SUBSYSTEM,
                                    logger);

    Architecture::Type architecture = master_node_process->read_device_architecture();
    EXPECT_TRUE(architecture != Architecture::Type::UNKNOWN);
    ResourceMonitor* resource_monitor = new ResourceMonitor(
        architecture, ResourceMonitor::Mode::PROCESS, master_node_process->get_root_diagnostic());

    Diagnostic::DiagnosticDefinition diag = resource_monitor->init();
    logger->log_diagnostic(diag);
    printf("%s\n", resource_monitor->pretty(resource_monitor->get_resourceinfo()).c_str());
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    double run_time = 0.0;
    double dt = 1.0;
    while (run_time <= TIME_TO_RUN) {
        diag = resource_monitor->update(dt);
        // logger->log_diagnostic(diag);
        // EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        usleep(dt * 1000000.0);
        run_time += dt;
    }
    delete resource_monitor;
    delete master_node_process;
    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}