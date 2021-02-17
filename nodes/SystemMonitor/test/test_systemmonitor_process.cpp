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
void print(std::map<std::string, SystemMonitorProcess::Task> task_list) {
    printf("--- Task List ---\n");
    if (task_list.size() == 0) {
        printf("\tNo Tasks Defined!\n");
        return;
    }
    std::map<std::string, SystemMonitorProcess::Task>::iterator it = task_list.begin();
    int i = 0;
    while (it != task_list.end()) {
        printf("\t[%d/%d] Type: %d Name: %s Rx: %4.2f\n",
               (uint16_t)i + 1,
               (uint16_t)task_list.size(),
               (uint8_t)it->second.type,
               it->second.node_name.c_str(),
               it->second.last_heartbeat_delta);
        i++;
        ++it;
    }
}
TEST(BasicTest, Conversions) {
    Logger* logger = new Logger("INFO", "UnitTestSystemMonitorProcess");
    SystemMonitorProcessTester* tester = new SystemMonitorProcessTester;
    tester->initialize("UnitTestSystemMonitorProcess",
                       "UnitTestSystemMonitorProcess",
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
    return;
    // Test Pixel Conversion
    uint16_t screen_width = 2000;
    uint16_t screen_height = 1000;
    {
        WindowManager::ScreenCoordinatePerc coord_perc(0.0, 0.0, 100.0, 100.0);
        WindowManager::ScreenCoordinatePixel coord_pixel =
            SystemMonitorProcess::convertCoordinate(coord_perc, screen_width, screen_height);
        printf("%d %d %d %d\n",
               coord_pixel.start_x_pix,
               coord_pixel.start_y_pix,
               coord_pixel.width_pix,
               coord_pixel.height_pix);
        EXPECT_TRUE(coord_pixel.start_x_pix == 0);
        EXPECT_TRUE(coord_pixel.start_y_pix == 0);
        EXPECT_TRUE(coord_pixel.width_pix == 2000);
        EXPECT_TRUE(coord_pixel.height_pix == 1000);
    }
    delete logger;
    delete tester;
}
TEST(BasicTest, TestOperation) {
    return;
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitorProcess");
    SystemMonitorProcessTester* tester = new SystemMonitorProcessTester;
    tester->initialize("UnitTestSystemMonitorProcess",
                       "UnitTestSystemMonitorProcess",
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
    Diagnostic::DiagnosticDefinition diag = tester->get_root_diagnostic();
    double dt = 0.1;
    double system_time = 0.0;
    {  // No nodes, heartbeats or anything else.
        std::vector<std::string> node_list;
        std::vector<std::string> heartbeat_list;
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        diag =
            tester->update_nodelist(node_list, heartbeat_list, new_heartbeat_topics_to_subscribe);
        print(tester->get_task_list());
        logger->log_diagnostic(diag);
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        EXPECT_TRUE(new_heartbeat_topics_to_subscribe.size() == 0);
        EXPECT_TRUE(tester->get_task_list().size() == 0);
    }
    {  // 1 new node, with a heartbeat for that node
        std::vector<std::string> node_list;
        node_list.push_back("/ABCNameSpace/Node1");
        std::vector<std::string> heartbeat_list;
        heartbeat_list.push_back("/ABCNameSpace/Node1/heartbeat");
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        diag =
            tester->update_nodelist(node_list, heartbeat_list, new_heartbeat_topics_to_subscribe);
        logger->log_diagnostic(diag);

        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        EXPECT_TRUE(new_heartbeat_topics_to_subscribe.size() == 1);
        std::map<std::string, SystemMonitorProcess::Task> task_list = tester->get_task_list();
        EXPECT_TRUE(task_list.size() == 1);
        uint16_t eros_node_count = 0;
        std::map<std::string, SystemMonitorProcess::Task>::iterator task_it = task_list.begin();
        while (task_it != task_list.end()) {
            if (task_it->second.type == SystemMonitorProcess::TaskType::EROS) {
                eros_node_count++;
            }
            ++task_it;
        }
        EXPECT_TRUE(eros_node_count == 1);
        diag = tester->update(0.1, system_time += dt);
        print(tester->get_task_list());
        task_list = tester->get_task_list();
        task_it = task_list.begin();
        while (task_it != task_list.end()) {
            EXPECT_TRUE(
                SystemMonitorProcess::isEqual(task_it->second.last_heartbeat_delta, dt, 1e-6));
            ++task_it;
        }
    }
    {
        // Same message as before, should not re-subscribe
        std::vector<std::string> node_list;
        node_list.push_back("/ABCNameSpace/Node1");
        std::vector<std::string> heartbeat_list;
        heartbeat_list.push_back("/ABCNameSpace/Node1/heartbeat");
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        diag =
            tester->update_nodelist(node_list, heartbeat_list, new_heartbeat_topics_to_subscribe);
        logger->log_diagnostic(diag);
        std::map<std::string, SystemMonitorProcess::Task> task_list = tester->get_task_list();
        std::map<std::string, SystemMonitorProcess::Task>::iterator task_it = task_list.begin();
        while (task_it != task_list.end()) {
            EXPECT_TRUE(
                SystemMonitorProcess::isEqual(task_it->second.last_heartbeat_delta, 0.0, 1e-6));
            ++task_it;
        }
        print(tester->get_task_list());
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        EXPECT_TRUE(new_heartbeat_topics_to_subscribe.size() == 0);
        EXPECT_TRUE(tester->get_task_list().size() == 1);
        diag = tester->update(0.1, system_time += dt);
        task_list = tester->get_task_list();
        task_it = task_list.begin();
        while (task_it != task_list.end()) {
            EXPECT_TRUE(
                SystemMonitorProcess::isEqual(task_it->second.last_heartbeat_delta, dt, 1e-6));
            ++task_it;
        }
    }

    // Test: New heartbeat found with no node (should add as EROS)
    {
        std::vector<std::string> node_list;
        std::vector<std::string> heartbeat_list;
        heartbeat_list.push_back("/ABCNameSpace/Node2/heartbeat");
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        diag =
            tester->update_nodelist(node_list, heartbeat_list, new_heartbeat_topics_to_subscribe);
        logger->log_diagnostic(diag);
        std::map<std::string, SystemMonitorProcess::Task> task_list = tester->get_task_list();
        print(tester->get_task_list());
        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        EXPECT_TRUE(new_heartbeat_topics_to_subscribe.size() == 1);
        EXPECT_TRUE(tester->get_task_list().size() == 2);
        diag = tester->update(0.1, system_time += dt);
        task_list = tester->get_task_list();
        print(tester->get_task_list());
    }

    // Test: New node found with no heartbeat found (should add as NON-EROS)
    {
        std::vector<std::string> node_list;
        node_list.push_back("/ABCNameSpace/Node3");
        std::vector<std::string> heartbeat_list;
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        diag =
            tester->update_nodelist(node_list, heartbeat_list, new_heartbeat_topics_to_subscribe);
        logger->log_diagnostic(diag);

        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        EXPECT_TRUE(new_heartbeat_topics_to_subscribe.size() == 0);
        std::map<std::string, SystemMonitorProcess::Task> task_list = tester->get_task_list();
        EXPECT_TRUE(task_list.size() == 3);
        uint16_t non_eros_node_count = 0;
        std::map<std::string, SystemMonitorProcess::Task>::iterator task_it = task_list.begin();
        while (task_it != task_list.end()) {
            if (task_it->second.type == SystemMonitorProcess::TaskType::NON_EROS) {
                non_eros_node_count++;
            }
            ++task_it;
        }
        EXPECT_TRUE(non_eros_node_count == 1);
        diag = tester->update(0.1, system_time += dt);
        print(tester->get_task_list());
    }
    // Test: New heartbeat found with existing node (should change from NON-EROS to EROS)
    {
        std::vector<std::string> node_list;
        std::vector<std::string> heartbeat_list;
        heartbeat_list.push_back("/ABCNameSpace/Node3/heartbeat");
        std::vector<std::string> new_heartbeat_topics_to_subscribe;
        diag =
            tester->update_nodelist(node_list, heartbeat_list, new_heartbeat_topics_to_subscribe);
        logger->log_diagnostic(diag);

        EXPECT_TRUE(diag.level <= Level::Type::NOTICE);
        EXPECT_TRUE(new_heartbeat_topics_to_subscribe.size() == 1);
        std::map<std::string, SystemMonitorProcess::Task> task_list = tester->get_task_list();
        EXPECT_TRUE(task_list.size() == 3);
        uint16_t non_eros_node_count = 0;
        uint16_t eros_node_count = 0;
        std::map<std::string, SystemMonitorProcess::Task>::iterator task_it = task_list.begin();
        while (task_it != task_list.end()) {
            if (task_it->second.type == SystemMonitorProcess::TaskType::NON_EROS) {
                non_eros_node_count++;
            }
            else if (task_it->second.type == SystemMonitorProcess::TaskType::EROS) {
                eros_node_count++;
            }
            ++task_it;
        }
        EXPECT_TRUE(non_eros_node_count == 0);
        EXPECT_TRUE(eros_node_count == 3);
        diag = tester->update(0.1, system_time += dt);
        print(tester->get_task_list());
    }
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}