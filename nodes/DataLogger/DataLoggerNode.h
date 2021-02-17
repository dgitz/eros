/*! \file DataLoggerNode.h
 */
#ifndef DataLoggerNode_H
#define DataLoggerNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
#include <rosbag/recorder.h>
// ROS Messages
#include <std_msgs/Empty.h>
// Project
#include <eros/BaseNode.h>

#include "DataLoggerProcess.h"

/*! \class DataLoggerNode DataLoggerNode.h "DataLoggerNode.h"
 *  \brief A Node that can be used to collect bag files.  Configured as either always logging to
 * disk, or snapshot mode where it will log to ram and write to disk when a snapshot trigger is
 * received.*/
class DataLoggerNode : public BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "datalogger_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 1;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 15-Feb-2021";

    /*! \brief What System this Node falls under.*/
    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ENTIRE_SYSTEM;

    /*! \brief What Component this Node falls under.*/
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::DIAGNOSTIC;
    DataLoggerNode();
    ~DataLoggerNode();
    DataLoggerProcess* get_process() {
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
    void snapshot_trigger_Callback(const std_msgs::Empty::ConstPtr& t_msg);
    void run_logger(DataLoggerNode* node);
    void cleanup();

   private:
    Diagnostic::DiagnosticDefinition read_launchparameters();
    DataLoggerProcess* process;
    ros::Subscriber snapshot_trigger_sub;
};

#endif  // DataLoggerNode_H