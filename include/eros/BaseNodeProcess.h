#ifndef EROS_BASENODEPROCESS_H
#define EROS_BASENODEPROCESS_H
// BaseNodeProcess class
// C System Files
#include <stdlib.h>
#include <sys/time.h>
// C++ System Files
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <eigen3/Eigen/Dense>

#include "boost/date_time/posix_time/posix_time.hpp"
// ROS Base Functionality
#include "ros/time.h"
// ROS Messages
#include <eros/command.h>
#include <eros/diagnostic.h>
#include <eros/heartbeat.h>
#include <eros/resource.h>
#include <eros/uptime.h>
// Project
#include "Diagnostic.h"
#include "Logger.h"
#include "eROS_Definitions.h"

/*! \class BaseNodeProcess BaseNodeProcess.h "BaseNodeProcess.h"
 *  \brief This is a BaseNodeProcess class.  All NodeProcess should be a derived class from this
 * BaseNodeProcess Class. */
class BaseNodeProcess
{
   public:
    BaseNodeProcess() : logger(nullptr), base_node_name("") {
    }
    virtual ~BaseNodeProcess() {
    }
    // Constants
    // Enums
    // Structs
    // Initialization Functions

    /*! \brief Initializes Process.  Should be called right after instantiating variable. */
    void initialize(std::string t_base_node_name,
                    std::string t_node_name,
                    std::string t_hostname,
                    System::MainSystem t_system,
                    System::SubSystem t_subsystem,
                    System::Component t_component,
                    Logger* _logger) {
        base_node_name = t_base_node_name;
        node_state = Node::State::INITIALIZING;
        unittest_running = false;
        run_time = 0.0;
        diagnostic_helper.initialize(t_hostname, t_node_name, t_system, t_subsystem, t_component);
        ready_to_arm = false;
        logger = _logger;
    }

    /*! \brief Derived Process Initialization */
    virtual Diagnostic::DiagnosticDefinition finish_initialization() = 0;
    /*! \brief Resets Process. Used for counters, timers, etc.*/
    virtual void reset() = 0;
    // Update Functions

    /*! \brief Update function must be implemented in Derived Process.  This is used for all state
     * machine logic, etc. */

    virtual Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time) = 0;

    // Attribute Functions
    Node::State get_nodestate() {
        return node_state;
    }
    double get_runtime() {
        return run_time;
    }
    bool get_ready_to_arm() {
        return ready_to_arm;
    }
    std::vector<Diagnostic::DiagnosticDefinition> get_diagnostics() {
        return diagnostic_helper.get_diagnostics();
    }
    double getROSTime() {
        return ros_time;
    }

    // Message Functions
    virtual std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg) = 0;

    // Support Functions
    /*! \brief Must be implemented in Derived Process.  Used for diagnostic testing LEVEL2 and for
     * basic checking of different variables, if they are initialized, etc. */
    virtual std::vector<Diagnostic::DiagnosticDefinition> check_programvariables() = 0;
    /*! \brief Runs Unit Test on Derived Process. */
    std::vector<Diagnostic::DiagnosticDefinition> run_unittest();
    ros::Time convert_time(struct timeval t);
    ros::Time convert_time(double t);

    std::string exec(const char* cmd, bool wait_for_result);
    // Printing Functions
    void print_message(std::string level,
                       std::string time_str,
                       std::string filename,
                       int line_number,
                       std::string msg);
    Logger* get_logger() {
        return logger;
    }

   protected:
    Logger* logger;
    std::string base_node_name;
    Node::State node_state;
    Diagnostic diagnostic_helper;
    Diagnostic::DiagnosticDefinition base_update(double t_dt, double t_ros_time);
    bool request_statechange(Node::State newstate);
    eros::command convert_fromptr(const eros::command::ConstPtr& t_ptr);
    eros::diagnostic convert_fromptr(const eros::diagnostic::ConstPtr& t_ptr);

    bool unittest_running;
    bool ready_to_arm;

   private:
    double run_time, ros_time;
};

#endif  // EROS_BASENODEPROCESS_H