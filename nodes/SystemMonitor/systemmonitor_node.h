/*! \file systemmonitor_node.h
 */
#ifndef SYSTEMMONITORNODE_H
#define SYSTEMMONITORNODE_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project

#include <eros/BaseNode.h>

#include "SystemMonitorProcess.h"

/*! \class SystemMonitorNode SystemMonitorNode.h "SystemMonitorNode.h"
 *  \brief */
class SystemMonitorNode : public BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "system_monitor";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 1;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 14-Feb-2021";

    /*! \brief What System this Node falls under.*/
    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::REMOTE_CONTROL;

    /*! \brief What Subsystem this Node falls under.*/
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ROBOT_MONITOR;

    /*! \brief What Component this Node falls under.*/
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::DIAGNOSTIC;
    SystemMonitorNode();
    ~SystemMonitorNode();
    SystemMonitorProcess* get_process() {
        return process;
    }
    bool start(int argc, char** argv);
    bool init_screen();
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
    void heartbeat_Callback(const eros::heartbeat::ConstPtr& msg);

   private:
    std::vector<ros::Subscriber> heartbeat_subs;
    Diagnostic::DiagnosticDefinition read_launchparameters();
    SystemMonitorProcess* process;
};

#endif  // SYSTEMMONITORNODE_H