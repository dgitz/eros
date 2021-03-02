/*! \file test_diagnostics.cpp
 */
#include <eros/BaseNode.h>
#include <eros/BaseNodeProcess.h>
#include <gtest/gtest.h>
#include <stdio.h>
static std::string get_hostname() {
    char name[1024];
    name[1023] = '\0';
    gethostname(name, 1023);
    return std::string(name);
}
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
    void cleanup() {
        base_cleanup();
    }

    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time) {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        diag = base_update(t_dt, t_ros_time);
        return diag;
    }
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg) {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        std::vector<Diagnostic::DiagnosticDefinition> diag_list;
        (void)msg;
        return diag_list;
    }
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables() {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        std::vector<Diagnostic::DiagnosticDefinition> diag_list;
        return diag_list;
    }
};
TEST(BasicTest, TestOperation_BaseNodeProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    json json_obj = tester->read_configuration(tester->get_hostname());
    printf("host: %s\n", tester->get_hostname().c_str());
    EXPECT_TRUE(json_obj.size() > 0);
    printf("json:%s\n", json_obj.dump().c_str());
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}