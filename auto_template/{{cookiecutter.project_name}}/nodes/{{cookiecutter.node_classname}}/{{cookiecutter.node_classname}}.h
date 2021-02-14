/*! \file {{cookiecutter.node_classname}}.h
 */
#ifndef {{cookiecutter.node_classname}}_H
#define {{cookiecutter.node_classname}}_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "{{cookiecutter.process_classname}}.h"

/*! \class {{cookiecutter.node_classname}} {{cookiecutter.node_classname}}.h "{{cookiecutter.node_classname}}.h"
 *  \brief */
class {{cookiecutter.node_classname}} : public BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "{{cookiecutter.node_name_binary}}";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 0;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 12-Feb-2021";

    /*! \brief What System this Node falls under.*/
    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::{{cookiecutter.system}};

    /*! \brief What Subsystem this Node falls under.*/
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::{{cookiecutter.subsystem}};

    /*! \brief What Component this Node falls under.*/
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::{{cookiecutter.component}};
    {{cookiecutter.node_classname}}();
    ~{{cookiecutter.node_classname}}();
    {{cookiecutter.process_classname}}* get_process() {
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
    {{cookiecutter.process_classname}}* process;
};

#endif  // {{cookiecutter.node_classname}}_H
