/*! \file BaseNode.h
 */
#ifndef EROS_BASENODE_H
#define EROS_BASENODE_H
// Base class
// C System Files
#include <unistd.h>

#include <csignal>

// C++ System Files
#include <iostream>
#include <thread>

// ROS Base Functionality
#include "ros/ros.h"
#include "ros/time.h"

// ROS Messages
#include <std_msgs/Bool.h>
// Project
#include <eros/BaseNodeProcess.h>

/*! \class BaseNode BaseNode.h "BaseNode.h"
 *  \brief This is a BaseNode class.  All Nodes should be a derived class from this Base Class.*/
class BaseNode
{
   public:
    BaseNode()
        : diagnostic(),
          n(),
          host_name(),
          firmware_version(),
          base_node_name(""),
          node_name(""),
          logger(nullptr),
          logger_initialized(false),
          ros_rate(-1.0),
          loop1_enabled(false),
          loop1_rate(-1.0),
          loop2_enabled(false),
          loop2_rate(-1.0),
          loop3_enabled(false),
          loop3_rate(-1.0),
          verbosity_level("DEBUG"),
          require_pps_to_start(false),
          pps_received(false),
          rand_delay_sec(0.0) {
    }
    virtual ~BaseNode() {
    }
    // Constants

    // Enums

    // Structs

    // Initialization Functions
    /*! \brief Set Node Base Name.  This will be the same for every instance of the node, and is
     * independent on where the node is run. This value is equivelant to the "type" field in the
     * launch file.
     */
    void set_basenodename(std::string t_base_node_name);

    /*! \brief Initializes Node Root Diagnostic. */
    void initialize_diagnostic(System::MainSystem t_system,
                               System::SubSystem t_subsystem,
                               System::Component t_component);
    /*! \brief Initializes firmware based on Major, Minor, Build Number and Description. */
    void initialize_firmware(uint16_t t_major_version,
                             uint16_t t_minor_version,
                             uint16_t t_build_number,
                             std::string t_description);

    void set_nodename(std::string t_node_name) {
        node_name = t_node_name;
    }

    /*! \brief Start the Node. */
    virtual bool start(int argc, char **argv) = 0;

    /*! \brief Pre-initialization of node.  This section will create the default pub/subs for the
     * node, along with the logger. */
    Diagnostic::DiagnosticDefinition preinitialize_basenode(int argc, char **argv);
    void set_loop1_rate(double t_rate) {
        loop1_rate = t_rate;
        loop1_enabled = true;
    }
    void set_loop2_rate(double t_rate) {
        loop2_rate = t_rate;
        loop2_enabled = true;
    }
    void set_loop3_rate(double t_rate) {
        loop3_rate = t_rate;
        loop3_enabled = true;
    }

    // Update Functions
    /*! \brief Main Node update section, will call all derived Node Loop Functions, and publish base
     * pubs such as firmware, heartbeat, etc. */
    bool update(Node::State node_state);
    /*! \brief Run code in Loop1 Function.  Loop1 Must be Implemented in Derived Node.*/
    virtual bool run_loop1() = 0;
    /*! \brief Run code in Loop2 Function.  Loop2 Must be Implemented in Derived Node.*/
    virtual bool run_loop2() = 0;
    /*! \brief Run code in Loop3 Function.  Loop3 Must be Implemented in Derived Node.*/
    virtual bool run_loop3() = 0;
    /*! \brief Run code at .01 Hz.  Loop .01Hz Must be Implemented in Derived Node.*/
    virtual bool run_001hz() = 0;
    /*! \brief Run code at .1 Hz Function.  Loop .1Hz Must be Implemented in Derived Node.*/
    virtual bool run_01hz() = 0;
    /*! \brief Run code at .1 Hz Function with a small random delay to prevent multiple things
     running all at the same time. .1Hz Noisy Must be Implemented in Derived Node.*/
    virtual bool run_01hz_noisy() = 0;
    /*! \brief Run code at .1 Hz. Loop 1Hz Must be Implemented in Derived Node.*/
    virtual bool run_1hz() = 0;
    /*! \brief Run code at 10 Hz. Loop 10Hz Must be Implemented in Derived Node.*/
    virtual bool run_10hz() = 0;
    /*! \brief Thread Loop Must be Implemented in Derived Node.*/
    virtual void thread_loop() = 0;

    // Attribute Functions
    std::string get_basenodename() {
        return base_node_name;
    }
    std::string get_nodename() {
        return node_name;
    }
    std::string get_verbositylevel() {
        return verbosity_level;
    }
    std::string get_hostname() {
        return std::string(host_name);
    }
    boost::shared_ptr<ros::NodeHandle> get_nodehandle() {
        return n;
    }
    Logger *get_logger() {
        return logger;
    }

    // Utility Functions
    /*! \brief Measures time delay between 2 ros::Time timestamps.
     *  Generally, if wanting to measure the time from now to a previous mark,
     * the current timestamp should be the first parameter and the previous mark should be the 2nd
     * parameter.
     */
    double measure_time_diff(ros::Time t_timer_a, ros::Time t_timer_b) {
        double etime = t_timer_a.toSec() - t_timer_b.toSec();
        return etime;
    }

    // Message Functions
    /*! \brief Handles receiving the 1 PPS Msg. */
    void new_ppsmsg(const std_msgs::Bool::ConstPtr &t_msg);

    // Destructors
    virtual void cleanup() = 0;
    void base_cleanup();

   protected:
    /*! \brief Get Base Launch parameters, which includes loop rates, verbosity, etc. */
    Diagnostic::DiagnosticDefinition read_baselaunchparameters();
    Diagnostic::DiagnosticDefinition diagnostic;

    boost::shared_ptr<ros::NodeHandle> n;
    char host_name[1024];
    Firmware firmware_version;
    std::string base_node_name;
    std::string node_name;
    ros::Publisher state_pub;
    ros::Publisher heartbeat_pub;
    eros::heartbeat heartbeat;
    Logger *logger = nullptr;
    bool logger_initialized;
    double ros_rate;

    ros::Time boot_time;
    ros::Time last_001hz_timer;
    ros::Time last_01hz_timer;
    ros::Time last_01hz_noisy_timer;
    ros::Time last_1hz_timer;
    ros::Time last_10hz_timer;
    bool loop1_enabled;
    ros::Time last_loop1_timer;
    double loop1_rate;

    bool loop2_enabled;
    ros::Time last_loop2_timer;
    double loop2_rate;

    bool loop3_enabled;
    double loop3_rate;
    ros::Time last_loop3_timer;

    std::string verbosity_level;
    bool require_pps_to_start;
    bool pps_received;
    double rand_delay_sec;

   private:
};

#endif  // EROS_BASENODE_H