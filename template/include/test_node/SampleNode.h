#ifndef SAMPLENODE_H
#define SAMPLENODE_H
#include <eros/BaseNode.h>

#include "SampleNodeProcess.h"
class SampleNode : public BaseNode
{
   public:
    const std::string BASE_NODE_NAME = "sample_node";

    const uint8_t MAJOR_RELEASE_VERSION = 0;
    const uint8_t MINOR_RELEASE_VERSION = 0;
    const uint8_t BUILD_NUMBER = 0;
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 12-Feb-2021";

    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::SIMROVER;
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ROBOT_CONTROLLER;
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::CONTROLLER;
    SampleNode();
    ~SampleNode();
    SampleNodeProcess* get_process() {
        return process;
    }
    bool start(int argc, char** argv);
    Diagnostic::DiagnosticDefinition finish_initialization();
    bool run_loop1();
    bool run_loop2();
    bool run_loop3();
    bool run_001hz();
    bool run_01hz();
    bool run_01hz_noisy();
    bool run_1hz();
    bool run_10hz();
    void thread_loop();
    void cleanup();

   private:
    Diagnostic::DiagnosticDefinition read_launchparameters();
    SampleNodeProcess* process;
};

#endif  // SAMPLENODE_H