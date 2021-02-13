/*! \file SampleNode.h
 */
#ifndef SAMPLENODE_H
#define SAMPLENODE_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "SampleNodeProcess.h"

/*! \class SampleNode SampleNode.h "SampleNode.h"
 *  \brief This is a SampleNode class illustrating usage of the EROS BaseNodeProcess and EROS
 * BaseNode..*/
class SampleNode : public BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "sample_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 0;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 12-Feb-2021";

    /*! \brief What System this Node falls under.*/
    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::SIMROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ROBOT_CONTROLLER;

    /*! \brief What Component this Node falls under.*/
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