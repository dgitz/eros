/*! \file MasterNode.h
 */
#ifndef MasterNode_H
#define MasterNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>

#include "MasterNodeProcess.h"
using namespace eros;
namespace eros_nodes {
/*! \class MasterNode MasterNode.h "MasterNode.h"
 *  \brief */
class MasterNode : public BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "master_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 1;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 1;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 22-Feb-2021";

    /*! \brief What System this Node falls under.*/
    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ENTIRE_SYSTEM;

    /*! \brief What Component this Node falls under.*/
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::CONTROLLER;
    MasterNode();
    ~MasterNode();
    MasterNodeProcess* get_process() {
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

    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    bool device_service(eros::srv_device::Request& req, eros::srv_device::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);

   private:
    Diagnostic::DiagnosticDefinition read_launchparameters();
    MasterNodeProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
    ros::ServiceServer device_server_srv;
    ResourceMonitor* resource_available_monitor;
    ros::Publisher resource_available_pub;
    ros::Publisher loadfactor_pub;
};
}  // namespace eros_nodes
#endif  // MasterNode_H
