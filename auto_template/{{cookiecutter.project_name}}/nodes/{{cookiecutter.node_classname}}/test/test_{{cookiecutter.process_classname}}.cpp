/*! \file test_{{cookiecutter.process_classname}}.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../{{cookiecutter.process_classname}}.h"

class {{cookiecutter.process_classname}}Tester : public {{cookiecutter.process_classname}}
{
   public:
    {{cookiecutter.process_classname}}Tester() {
    }
    ~{{cookiecutter.process_classname}}Tester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger =
        new Logger("DEBUG", "/home/robot/var/log/output", "UnitTest{{cookiecutter.process_classname}}");
    {{cookiecutter.process_classname}}Tester* tester = new {{cookiecutter.process_classname}}Tester;
    tester->initialize("UnitTest{{cookiecutter.process_classname}}",
                       "UnitTest{{cookiecutter.process_classname}}",
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
  
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
