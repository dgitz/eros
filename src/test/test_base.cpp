/*! \file test_diagnostics.cpp
 */
#include <eros/BaseNodeProcess.h>
#include <gtest/gtest.h>
#include <stdio.h>

class BaseNodeProcessTester : public BaseNodeProcess
{
   public:
    BaseNodeProcessTester() {
        Diagnostic diag_helper;
        diag_helper.initialize("UnitTestDevice",
                               "UnitTestNode-BaseNodeProcess",
                               System::MainSystem::SIMROVER,
                               System::SubSystem::ENTIRE_SYSTEM,
                               System::Component::ENTIRE_SUBSYSTEM);
        std::unique_ptr<Logger> logger(new Logger(
            "INFO", "/home/robot/var/log/output/", diag_helper.get_root_diagnostic().node_name));

        initialize(diag_helper.get_root_diagnostic().node_name,
                   diag_helper.get_root_diagnostic().node_name,
                   diag_helper.get_root_diagnostic().device_name,
                   diag_helper.get_root_diagnostic().system,
                   diag_helper.get_root_diagnostic().subsystem,
                   diag_helper.get_root_diagnostic().component,
                   logger);
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
TEST(BasicTest, TestOperation) {
    BaseNodeProcessTester tester;
    EXPECT_TRUE(false);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}