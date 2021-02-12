/*! \file test_diagnostics.cpp
 */
#include <eros/BaseNode.h>
#include <eros/BaseNodeProcess.h>
#include <gtest/gtest.h>
#include <stdio.h>

class BaseNodeProcessTester : public BaseNodeProcess
{
   public:
    BaseNodeProcessTester() {
    }
    ~BaseNodeProcessTester() {
    }

    Diagnostic::DiagnosticDefinition finish_initialization() {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        return diag;
    }

    void reset() {
    }

    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time) {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        diag = base_update(t_dt, t_ros_time);
        return diag;
    }
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg) {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        std::vector<Diagnostic::DiagnosticDefinition> diag_list;

        eros::command cmd = convert_fromptr(t_msg);
        return diag_list;
    }
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables() {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        std::vector<Diagnostic::DiagnosticDefinition> diag_list;
        return diag_list;
    }
};
class BaseNodeTester : public BaseNode
{
   public:
    const std::string BASE_NODE_NAME = "unittester_node";

    const uint8_t MAJOR_RELEASE_VERSION = 4;
    const uint8_t MINOR_RELEASE_VERSION = 1;
    const uint8_t BUILD_NUMBER = 0;
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 3-Aug-2019";

    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::SIMROVER;
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ROBOT_CONTROLLER;
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::CONTROLLER;
    BaseNodeTester() {
    }

    bool start(int argc, char** argv) {
        process = new BaseNodeProcessTester();
        set_basenodename(BASE_NODE_NAME);
        initialize_firmware(
            MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
        Logger* logger = new Logger("INFO", "/home/robot/var/log/output/", BASE_NODE_NAME);
        process->initialize(get_basenodename(),
                            get_nodename(),
                            get_hostname(),
                            DIAGNOSTIC_SYSTEM,
                            DIAGNOSTIC_SUBSYSTEM,
                            DIAGNOSTIC_COMPONENT,
                            logger);
        return true;
    }
    BaseNodeProcessTester* get_process() {
        return process;
    }
    ~BaseNodeTester() {
    }
    bool run_loop1() {
        return false;
    }
    bool run_loop2() {
        return false;
    }
    bool run_loop3() {
        return false;
    }
    bool run_001hz() {
        return false;
    }
    bool run_01hz() {
        return false;
    }
    bool run_01hz_noisy() {
        return false;
    }
    bool run_1hz() {
        return false;
    }
    bool run_10hz() {
        return false;
    }
    void thread_loop() {
    }
    void cleanup() {
        delete logger;
        delete process;
    }

   private:
    BaseNodeProcessTester* process;
};
TEST(BasicTest, TestOperation_BaseNode) {
    BaseNodeTester* tester = new BaseNodeTester;
    int argc;
    char** argv;
    EXPECT_TRUE(tester->start(argc, argv));
    EXPECT_TRUE(tester->update(Node::State::RUNNING));

    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}