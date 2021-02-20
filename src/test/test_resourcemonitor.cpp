/*! \file test_resourcemonitor.cpp
 */
#include <eros/Logger.h>
#include <eros/ResourceMonitor.h>
#include <gtest/gtest.h>

TEST(BasicTest, TestOperation_Node) {
    const double TIME_TO_RUN = 1000.0;

    Diagnostic::DiagnosticDefinition diag;
    diag.node_name = "UnitTestResourceMonitor";
    diag.device_name = "UnitTest";
    Logger* logger = new Logger("INFO", diag.node_name);
    diag.system = System::MainSystem::ROVER;
    diag.subsystem = System::SubSystem::ROBOT_CONTROLLER;
    diag.component = System::Component::CONTROLLER;
    diag.type = Diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    ResourceMonitor* resource_monitor =
        new ResourceMonitor(ResourceMonitor::Mode::PROCESS, diag, logger);
    diag = resource_monitor->init();
    EXPECT_TRUE(resource_monitor->get_architecture() != Architecture::Type::UNKNOWN);
    logger->log_diagnostic(diag);
    printf("%s\n", resource_monitor->pretty(resource_monitor->get_resourceinfo()).c_str());
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    double run_time = 0.0;
    double dt = 1.0;
    while (run_time <= TIME_TO_RUN) {
        diag = resource_monitor->update(dt);
        printf("%s\n", resource_monitor->pretty(resource_monitor->get_resourceinfo()).c_str());
        logger->log_diagnostic(diag);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

        logger->log_info("Time: " + std::to_string(run_time) +
                         " (sec) Finish: " + std::to_string(TIME_TO_RUN));
        usleep(dt * 1000000.0);
        run_time += dt;
    }
    delete resource_monitor;
    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
