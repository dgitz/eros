/*! \file DiagnosticNode.h
 */
#ifndef DiagnosticNode_H
#define DiagnosticNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "DiagnosticNodeProcess.h"
using namespace eros;
namespace eros_nodes {
/*! \class DiagnosticNode DiagnosticNode.h "DiagnosticNode.h"
 *  \brief */
class DiagnosticNode : public BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "diagnostic_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 3;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 16-March-2021";

    /*! \brief What System this Node falls under.*/
    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ROBOT_MONITOR;

    /*! \brief What Component this Node falls under.*/
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::DIAGNOSTIC;
    DiagnosticNode();
    ~DiagnosticNode();
    DiagnosticNodeProcess* get_process() {
        return process;
    }
    bool start();
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

    Diagnostic::DiagnosticDefinition rescan_nodes();
    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    bool system_diagnostics_service(eros::srv_get_diagnostics::Request& req,
                                    eros::srv_get_diagnostics::Response& res);
    void command_Callback(const eros::command::ConstPtr& t_msg);
    void diagnostic_Callback(const eros::diagnostic::ConstPtr& t_msg);

   private:
    Diagnostic::DiagnosticDefinition read_launchparameters();
    DiagnosticNodeProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
    std::vector<ros::Subscriber> diagnostic_subs;
    ros::ServiceServer system_diagnostics_srv;
};
}  // namespace eros_nodes
#endif  // DiagnosticNode_H