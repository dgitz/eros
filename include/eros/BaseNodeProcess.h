/*! \file BaseNodeProcess.h
 */

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

// ROS Services
#include <eros/srv_firmware.h>
#include <eros/srv_get_diagnostics.h>
#include <eros/srv_logger_level.h>
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
    BaseNodeProcess()
        : logger(nullptr),
          base_node_name(""),
          node_state(Node::State::UNKNOWN),
          diagnostic_helper(),
          unittest_running(false),
          ready_to_arm(false),
          run_time(0.0),
          system_time(0.0) {
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
        diagnostic_helper.initialize(t_hostname, t_node_name, t_system, t_subsystem, t_component);
        logger = _logger;
    }
    bool enable_diagnostics(std::vector<Diagnostic::DiagnosticType> diagnostic_types) {
        return diagnostic_helper.enable_diagnostics(diagnostic_types);
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
    Diagnostic::DiagnosticDefinition get_root_diagnostic() {
        return diagnostic_helper.get_root_diagnostic();
    }
    std::vector<Diagnostic::DiagnosticDefinition> get_diagnostics() {
        return diagnostic_helper.get_diagnostics();
    }
    double get_system_time() {
        return system_time;
    }
    double get_run_time() {
        return run_time;
    }

    Logger* get_logger() {
        return logger;
    }

    //! Request a Node State Change
    /*!
      \param newstate The state to be changed to.
      \return If the state change was successful (true) or not (false)
    */
    bool request_statechange(Node::State newstate);

    // Message Functions
    virtual std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg) = 0;

    // Support Functions
    /*! \brief Must be implemented in Derived Process.  Used for diagnostic testing LEVEL2 and for
     * basic checking of different variables, if they are initialized, etc. */
    virtual std::vector<Diagnostic::DiagnosticDefinition> check_programvariables() = 0;
    /*! \brief Runs Unit Test on Derived Process. */
    std::vector<Diagnostic::DiagnosticDefinition> run_unittest();

    //! Convert struct timeval to ros::Time
    /*!
      \param t Standard timeval object
      \return Time converted to ros::Time
    */
    ros::Time convert_time(struct timeval t);

    //! Convert time as a float to ros::Time
    /*!
      \param t timestamp in seconds.
      \return Time converted to ros::Time
    */
    ros::Time convert_time(double t);

    //! Execute a command
    /*!
      \param cmd The command to execute
      \param wait_for_results If function should return results or not.
      \return The result of the command
    */
    std::string exec(const char* cmd, bool wait_for_result);

    static bool isEqual(double a, double b, double eps);
    // Printing Functions

    // Destructors
    virtual void cleanup() = 0;
    void base_cleanup();

   protected:
    //! Convert eros::heartbeat message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    eros::heartbeat convert_fromptr(const eros::heartbeat::ConstPtr& t_ptr);

    //! Convert eros::command message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    eros::command convert_fromptr(const eros::command::ConstPtr& t_ptr);

    //! Convert eros::diagnostic message (as received via a ROS Node) to the regular datatype
    /*!
      \param t_ptr The pointer to the object
      \return The object
    */
    eros::diagnostic convert_fromptr(const eros::diagnostic::ConstPtr& t_ptr);

    //! Base Update Function of all Node Process Classes.
    /*!
      \param t_dt The delta in the sample time.
      \param t_system_time The current system time.
      \return A Diagnostic reflecting the status of the function.
    */
    Diagnostic::DiagnosticDefinition base_update(double t_dt, double t_system_time);

    Logger* logger;
    std::string base_node_name;
    Node::State node_state;
    Diagnostic diagnostic_helper;

    bool unittest_running;
    bool ready_to_arm;

   private:
    double run_time, system_time;
};

#endif  // EROS_BASENODEPROCESS_H