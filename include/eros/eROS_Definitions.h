/*! \file eROS_Definitions.h
 */

#ifndef EROSDEFINITIONS_H
#define EROSDEFINITIONS_H
#include <string>

/*! \class System
    \brief System class
    Holds System and lower level definitions based on the Hierarchy:
   System <- System::SubSystem <- System::Component
*/
class System
{
   public:
    enum class MainSystem {
        UNKNOWN = 0,        /*!< Uninitialized value. */
        ROVER = 1,          /*!< Rover System. */
        SIMROVER = 2,       /*!< Simulated Rover System. */
        REMOTE_CONTROL = 3, /*!< Remote Control System. */
        END_OF_LIST = 4     /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert System::MainSystem to human readable string
    /*!
      \param v System::MainSystem type
      \return The converted string.
    */
    static std::string MainSystemString(System::MainSystem v) {
        switch (v) {
            case System::MainSystem::UNKNOWN: return "UNKNOWN"; break;
            case System::MainSystem::ROVER: return "ROVER"; break;
            case System::MainSystem::SIMROVER: return "SIMROVER"; break;
            case System::MainSystem::REMOTE_CONTROL: return "REMOTE_CONTROL"; break;
            default: return MainSystemString(System::MainSystem::UNKNOWN); break;
        }
    }

    enum class SubSystem {
        UNKNOWN = 0,          /*!< Uninitialized value. */
        ENTIRE_SYSTEM = 1,    /*!< Applies to Entire System .*/
        ROBOT_CONTROLLER = 2, /*!< A Controller Subsystem. */
        ROBOT_MONITOR = 3,    /*!< A Monitor Subsystem */
        END_OF_LIST = 4       /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert System::SubSystem to human readable string
    /*!
      \param v System::SubSystem type
      \return The converted string.
    */
    static std::string SubSystemString(System::SubSystem v) {
        switch (v) {
            case System::SubSystem::UNKNOWN: return "UNKNOWN"; break;
            case System::SubSystem::ENTIRE_SYSTEM: return "ENTIRE_SYSTEM"; break;
            case System::SubSystem::ROBOT_CONTROLLER: return "ROBOT_CONTROLLER"; break;
            case System::SubSystem::ROBOT_MONITOR: return "ROBOT_MONITOR"; break;
            default: return SubSystemString(System::SubSystem::UNKNOWN); break;
        }
    }

    enum class Component {
        UNKNOWN = 0,          /*!< Uninitialized value. */
        ENTIRE_SUBSYSTEM = 1, /*!< Applies to entire SubSystem. */
        CONTROLLER = 2,       /*!< A Controller Component. */
        DIAGNOSTIC = 3,       /*!< A Diagnostic Component. */
        NAVIGATION = 4,       /*!< A Navigation Component, such as Navigation Planning, Navigation
                                 Execution, etc. */
        MAPPING = 5,          /*!< A Component used for Mapping. */
        LEARNING = 6,         /*!< A Component used for Machine Learning. */
        TARGETING = 7,        /*!< A Component used for Target Acquisition. */
        TIMING = 8,           /*!< A Component used for Timing, Time Synchronization, etc. */
        VISION = 9, /*!< A Component used for Perception, Image Capture, Video Capture, etc. */
        GPIO = 10,  /*!< A Component used for reading/writion Inputs and Outputs. */
        COMMUNICATION = 11, /*!< A Component used for Communication. */
        DYNAMICS = 12,      /*!< A Component used for monitoring system dynamics. */
        POWER = 13,         /*!< A Component used to monitor and report power state. */
        POSE = 14,          /*!< A Component used to compute Pose. */
        END_OF_LIST = 15    /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert System::Component to human readable string
    /*!
      \param v System::Component type
      \return The converted string.
    */
    static std::string ComponentString(System::Component v) {
        switch (v) {
            case System::Component::UNKNOWN: return "UNKNOWN"; break;
            case System::Component::ENTIRE_SUBSYSTEM: return "ENTIRE_SUBSYSTEM"; break;
            case System::Component::CONTROLLER: return "CONTROLLER"; break;
            case System::Component::DIAGNOSTIC: return "DIAGNOSTIC"; break;
            case System::Component::NAVIGATION: return "NAVIGATION"; break;
            case System::Component::MAPPING: return "MAPPING"; break;
            case System::Component::LEARNING: return "LEARNING"; break;
            case System::Component::TARGETING: return "TARGETING"; break;
            case System::Component::TIMING: return "TIMING"; break;
            case System::Component::VISION: return "VISION"; break;
            case System::Component::GPIO: return "GPIO"; break;
            case System::Component::COMMUNICATION: return "COMMUNICATION"; break;
            case System::Component::DYNAMICS: return "DYNAMICS"; break;
            case System::Component::POWER: return "POWER"; break;
            case System::Component::POSE: return "POSE"; break;
            default: return ComponentString(System::Component::UNKNOWN); break;
        }
    }
};
/*! \class Level
    \brief Level class

*/
class Level
{
   public:
    enum class Type {
        UNKNOWN = 0, /*!< Uninitialized value. */
        DEBUG = 1,   /*!< This Level is solely for development/debugging only. */
        INFO = 2,    /*!< This Level is purely for informational use only. */
        NOTICE = 3,  /*!< This Level is a higher form of information and does not imply that
                     // anything is wrong. */
        WARN = 4,    /*!< This Level implies that a program is not running as expected, but may
                     // continue to operate in a diminished capacity. */
        ERROR = 5,   /*!< This Level implies that a program will not initialize or some other kind
                     // of crash. */
        FATAL = 6,   /*!<   This Level implies that a program has failed so bad it can cause
                        injury   to itself or others. */
        END_OF_LIST = 7 /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert Level::Type to human readable string
    /*!
      \param v Level::Type type
      \return The converted string.
    */
    static std::string LevelString(Level::Type v) {
        switch (v) {
            case Level::Type::UNKNOWN: return "UNKNOWN"; break;
            case Level::Type::DEBUG: return "DEBUG"; break;    //
            case Level::Type::INFO: return "INFO"; break;      //
            case Level::Type::NOTICE: return "NOTICE"; break;  //
            case Level::Type::WARN: return "WARN"; break;      //
            case Level::Type::ERROR: return "ERROR"; break;    //
            case Level::Type::FATAL: return "FATAL"; break;    //
            default: return LevelString(Level::Type::UNKNOWN); break;
        }
    }
    static Level::Type LevelType(std::string level) {
        if (level == "DEBUG") {
            return Level::Type::DEBUG;
        }
        if (level == "INFO") {
            return Level::Type::INFO;
        }
        if (level == "NOTICE") {
            return Level::Type::NOTICE;
        }
        if (level == "WARN") {
            return Level::Type::WARN;
        }
        if (level == "ERROR") {
            return Level::Type::ERROR;
        }
        if (level == "FATAL") {
            return Level::Type::FATAL;
        }
        return Level::Type::UNKNOWN;
    }
};
/*! \class ArmDisarm
    \brief ArmDisarm class
*/
class ArmDisarm
{
   public:
    enum class Type {
        UNKNOWN = 0,            /*!< Uninitialized value. */
        ARMED = 1,              /*!< System is Armed. */
        DISARMED_CANNOTARM = 2, /*!< System is Disarmed, and Cannot Arm for some Reason. */
        DISARMED = 3,           /*!< System is Disarmed.  Can Arm if commanded to. */
        DISARMING = 4,          /*!< System is Disarming. */
        ARMING = 5,             /*!< System is Arming. */
        END_OF_LIST = 6         /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert ArmDisarm::Type to human readable string
    /*!
      \param v ArmDisarm::Type type
      \return The converted string.
    */
    static std::string ArmDisarmString(ArmDisarm::Type v) {
        switch (v) {
            case ArmDisarm::Type::UNKNOWN: return "UNKNOWN"; break;
            case ArmDisarm::Type::ARMED: return "ARMED"; break;
            case ArmDisarm::Type::DISARMED_CANNOTARM: return "DISARMED_CANNOTARM"; break;
            case ArmDisarm::Type::DISARMED: return "DISARMED"; break;
            case ArmDisarm::Type::DISARMING: return "DISARMING"; break;
            case ArmDisarm::Type::ARMING: return "ARMING"; break;
            default: return ArmDisarmString(ArmDisarm::Type::UNKNOWN); break;
        }
    }
};
/*! \class Command
    \brief Command class
*/
class Command
{
   public:
    enum class Type {
        UNKNOWN = 0,            /*!< Uninitialized value. */
        NONE = 1,               /*!< Don't perform a Command. */
        RUN_DIAGNOSTIC = 2,     /*!< Run a System Diagnostic. */
        ACQUIRE_TARGET = 3,     /*!< Search/Acquire a Target. */
        ARM = 4,                /*!< Arm the System. */
        DISARM = 5,             /*!< Disarm the System. */
        CONFIGURE = 6,          /*!< Perform a Configuration Command. */
        RUN = 7,                /*!< Perform regular system operation. */
        STOPMOVEMENT = 8,       /*!< Stop Moving (Safely). */
        DRIVE = 9,              /*!< Perform a manual Drive Command. */
        RESET = 10,             /*!< Reset the System. */
        SETLOGLEVEL = 11,       /*!< Change the Log Level. */
        TASKCONTROL = 12,       /*!< Change the Task Execution mode. */
        CALIBRATION = 13,       /*!< Perform a Calibration Command. */
        GENERATE_SNAPSHOT = 14, /*!< Generate a System Snapshot. */
        WAIT = 15,              /*!< Wait for further instructions. */
        END_OF_LIST = 16        /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert System::Type to human readable string
    /*!
      \param v Command::Type type
      \return The converted string.
    */
    static std::string CommandString(Command::Type v) {
        switch (v) {
            case Command::Type::UNKNOWN: return "UNKNOWN"; break;
            case Command::Type::NONE: return "NONE"; break;
            case Command::Type::RUN_DIAGNOSTIC: return "RUN_DIAGNOSTIC"; break;
            case Command::Type::ACQUIRE_TARGET: return "ACQUIRE_TARGET"; break;
            case Command::Type::ARM: return "ARM"; break;
            case Command::Type::DISARM: return "DISARM"; break;
            case Command::Type::CONFIGURE: return "CONFIGURE"; break;
            case Command::Type::RUN: return "RUN"; break;
            case Command::Type::STOPMOVEMENT: return "STOPMOVEMENT"; break;
            case Command::Type::DRIVE: return "DRIVE"; break;
            case Command::Type::RESET: return "RESET"; break;
            case Command::Type::SETLOGLEVEL: return "SETLOGLEVEL"; break;
            case Command::Type::TASKCONTROL: return "TASKCONTROL"; break;
            case Command::Type::CALIBRATION: return "CALIBRATION"; break;
            case Command::Type::GENERATE_SNAPSHOT: return "GENERATE_SNAPSHOT"; break;
            case Command::Type::WAIT: return "WAIT"; break;
            default: return CommandString(Command::Type::UNKNOWN); break;
        }
    }
};
/*! \class Node
    \brief Node class
*/
class Node
{
   public:
    enum class State {
        UNKNOWN = 0,      /*!< Uninitialized value. */
        START = 1,        /*!< Node has just started. */
        INITIALIZING = 2, /*!< Node is initializing. */
        INITIALIZED = 3,  /*!< Node has finished initializing. */
        RUNNING = 4,      /*!< Node is running. */
        PAUSED = 5,       /*!< Node is paused. */
        RESET = 6,        /*!< Node is reset. */
        NODATA = 7,       /*!< Node is missing data. */
        FINISHED = 8,     /*!< Node has finished cleanly. */
        CRASHED = 9,      /*!< Node has crashed. */
        END_OF_LIST = 10  /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert Node::State to human readable string
    /*!
      \param v Node::State type
      \return The converted string.
    */
    static std::string NodeStateString(Node::State v) {
        switch (v) {
            case Node::State::UNKNOWN: return "UNKNOWN"; break;
            case Node::State::START: return "START"; break;
            case Node::State::INITIALIZING: return "INITIALIZING"; break;
            case Node::State::INITIALIZED: return "INITIALIZED"; break;
            case Node::State::RUNNING: return "RUNNING"; break;
            case Node::State::PAUSED: return "PAUSED"; break;
            case Node::State::RESET: return "RESET"; break;
            case Node::State::NODATA: return "NODATA"; break;
            case Node::State::FINISHED: return "FINISHED"; break;
            case Node::State::CRASHED: return "CRASHED"; break;
            default: return NodeStateString(Node::State::UNKNOWN); break;
        }
    }
};
#endif  // EROSDEFINITIONS_H
