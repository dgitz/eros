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
class {{cookiecutter.node_classname}} : public eros::BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "{{cookiecutter.node_name_binary}}";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 5;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 16-July-2021";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::{{cookiecutter.system}};

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::{{cookiecutter.subsystem}};

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::{{cookiecutter.component}};
    {{cookiecutter.node_classname}}();
    ~{{cookiecutter.node_classname}}();
    {{cookiecutter.process_classname}}* get_process() {
        return process;
    }
    bool start();
    eros::eros_diagnostic::Diagnostic finish_initialization();
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

    bool changenodestate_service(eros::srv_change_nodestate::Request &req,
                             eros::srv_change_nodestate::Response &res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr &t_msg);

   private:
    eros::eros_diagnostic::Diagnostic read_launchparameters();
    {{cookiecutter.process_classname}}* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
};

#endif  // {{cookiecutter.node_classname}}_H
