/*! \file test_resourcemonitor.cpp
 */
#include <eros/Logger.h>
#include <eros/ResourceMonitor.h>
#include <gtest/gtest.h>
using namespace eros;
bool isEqual(double a, double b, double eps) {
    double dv = a - b;
    if (fabs(dv) < eps) {
        return true;
    }
    else {
        return false;
    }
}
TEST(BasicTest, TestOperation_Process) {
    const double TIME_TO_RUN = 3.0;

    Diagnostic::DiagnosticDefinition diag;
    diag.node_name = "UnitTestResourceMonitor";
    diag.device_name = "UnitTest";
    Logger* logger = new Logger("INFO", diag.node_name);
    diag.system = System::MainSystem::ROVER;
    diag.subsystem = System::SubSystem::ROBOT_CONTROLLER;
    diag.component = System::Component::CONTROLLER;
    diag.type = Diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    diag.level = Level::Type::INFO;
    ResourceMonitor* resource_monitor =
        new ResourceMonitor(ResourceMonitor::Mode::PROCESS, diag, logger);
    EXPECT_FALSE(resource_monitor->is_initialized());
    diag = resource_monitor->init();
    EXPECT_TRUE(resource_monitor->is_initialized());
    EXPECT_TRUE(resource_monitor->get_architecture() != Architecture::Type::UNKNOWN);
    logger->log_diagnostic(diag);
    printf("%s\n", resource_monitor->pretty(resource_monitor->get_resourceinfo()).c_str());
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    ResourceMonitor::ResourceInfo resourceInfo = resource_monitor->get_resourceinfo();

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
TEST(BasicTest, TestOperation_Device) {
    const double TIME_TO_RUN = 3.0;

    Diagnostic::DiagnosticDefinition diag;
    diag.node_name = "UnitTestResourceMonitor";
    diag.device_name = "UnitTest";
    Logger* logger = new Logger("INFO", diag.node_name);
    diag.system = System::MainSystem::ROVER;
    diag.subsystem = System::SubSystem::ROBOT_CONTROLLER;
    diag.component = System::Component::CONTROLLER;
    diag.type = Diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    diag.level = Level::Type::INFO;
    ResourceMonitor* resource_monitor =
        new ResourceMonitor(ResourceMonitor::Mode::DEVICE, diag, logger);
    EXPECT_FALSE(resource_monitor->is_initialized());
    diag = resource_monitor->init();
    EXPECT_TRUE(resource_monitor->is_initialized());
    EXPECT_TRUE(resource_monitor->get_architecture() != Architecture::Type::UNKNOWN);
    logger->log_diagnostic(diag);
    printf("%s\n", resource_monitor->pretty(resource_monitor->get_resourceinfo()).c_str());
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    ResourceMonitor::ResourceInfo resourceInfo = resource_monitor->get_resourceinfo();

    double run_time = 0.0;
    double dt = 1.0;
    while (run_time <= TIME_TO_RUN) {
        diag = resource_monitor->update(dt);
        printf("%s\n", resource_monitor->pretty(resource_monitor->get_resourceinfo()).c_str());
        std::vector<double> load_factor = resource_monitor->get_load_factor();
        EXPECT_TRUE(load_factor.size() == 3);
        printf("Load Factor: 1 Min: %4.2f 5 Min: %4.2f 15 Min: %4.2f\n",
               load_factor.at(0),
               load_factor.at(1),
               load_factor.at(2));
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
TEST(BasicOperation, NormalAPI) {
    Diagnostic::DiagnosticDefinition diag;
    diag.node_name = "UnitTestResourceMonitor";
    diag.device_name = "UnitTest";
    Logger* logger = new Logger("INFO", diag.node_name);
    diag.system = System::MainSystem::ROVER;
    diag.subsystem = System::SubSystem::ROBOT_CONTROLLER;
    diag.component = System::Component::CONTROLLER;
    diag.type = Diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    diag.level = Level::Type::INFO;
    ResourceMonitor* resource_monitor =
        new ResourceMonitor(ResourceMonitor::Mode::DEVICE, diag, logger);
    EXPECT_FALSE(resource_monitor->is_initialized());
    diag = resource_monitor->init();
    EXPECT_TRUE(resource_monitor->is_initialized());
    EXPECT_TRUE(resource_monitor->reset());

    delete resource_monitor;
    delete logger;
}
TEST(FailureTests, FailureCases) {
    Diagnostic::DiagnosticDefinition diag;
    diag.node_name = "UnitTestResourceMonitor";
    diag.device_name = "UnitTest";
    Logger* logger = new Logger("INFO", diag.node_name);
    diag.system = System::MainSystem::ROVER;
    diag.subsystem = System::SubSystem::ROBOT_CONTROLLER;
    diag.component = System::Component::CONTROLLER;
    diag.type = Diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    diag.level = Level::Type::INFO;
    ResourceMonitor* resource_monitor =
        new ResourceMonitor(ResourceMonitor::Mode::DEVICE, diag, logger);
    EXPECT_FALSE(resource_monitor->is_initialized());
    {  // Initialization Function Not Called
        diag = resource_monitor->update(0.1);
        EXPECT_EQ(diag.message, Diagnostic::Message::INITIALIZING_ERROR);
        EXPECT_TRUE(diag.level > Level::Type::NOTICE);
    }
    delete resource_monitor;
    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
