#ifndef EROSDEFINITIONS_H
#define EROSDEFINITIONS_H
#include <string>
class System
{
    public:
    enum class MainSystem
    {
        UNKNOWN=0,
        ROVER=1,
        SIMROVER=2,
        REMOTE_CONTROL=3,
        END_OF_LIST=4
    };
    static std::string MainSystemString(System::MainSystem v){
        switch(v)
        {
            case System::MainSystem::UNKNOWN: return "UNKNOWN"; break;
            case System::MainSystem::ROVER: return "ROVER"; break;
            case System::MainSystem::SIMROVER: return "SIMROVER"; break;
            case System::MainSystem::REMOTE_CONTROL: return "REMOTE_CONTROL"; break;
            default: return MainSystemString(System::MainSystem::UNKNOWN); break;
        }
    }

    enum class SubSystem{
        UNKNOWN=0,
        ENTIRE_SYSTEM=1,
        ROBOT_CONTROLLER=2,
        ROBOT_MONITOR=3,
        END_OF_LIST=4
    };
    static std::string SubSystemString(System::SubSystem v){
        switch(v)
        {
            case System::SubSystem::UNKNOWN: return "UNKNOWN"; break;
            case System::SubSystem::ENTIRE_SYSTEM: return "ENTIRE_SYSTEM"; break;
            case System::SubSystem::ROBOT_CONTROLLER: return "ROBOT_CONTROLLER"; break;
            case System::SubSystem::ROBOT_MONITOR: return "ROBOT_MONITOR"; break;
            default: return SubSystemString(System::SubSystem::UNKNOWN); break;
        }
    }

    enum class Component{
        UNKNOWN=0,
        ENTIRE_SUBSYSTEM=1,
        CONTROLLER=2,
        DIAGNOSTIC=3,
        NAVIGATION=4,
        MAPPING=5,
        LEARNING=6,
        TARGETING=7,
        TIMING=8,
        VISION=9,
        GPIO=10,
        COMMUNICATION=11,
        DYNAMICS=12,
        POWER=13,
        POSE=14,
        END_OF_LIST=15
    };
     static std::string ComponentString(System::Component v){
        switch(v)
        {
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

class Level
{
    public:
    enum class Type{
        UNKNOWN=0,
        DEBUG=1,
        INFO=2,
        NOTICE=3,
        WARN=4,
        ERROR=5,
        FATAL=6,
        END_OF_LIST=7
    };
    static std::string LevelString(Level::Type v){
        switch(v)
        {
            case Level::Type::UNKNOWN: return "UNKNOWN"; break;
            case Level::Type::DEBUG: return "DEBUG"; break; //This Level is solely for development/debugging only.
            case Level::Type::INFO: return "INFO"; break; //This Level is purely for informational use only.
            case Level::Type::NOTICE: return "NOTICE"; break; //This Level is a higher form of information and does not imply that anything is wrong.
            case Level::Type::WARN: return "WARN"; break; //This Level implies that a program is not running as expected, but may continue to operate in a diminished capacity.
            case Level::Type::ERROR: return "ERROR"; break; //This Level implies that a program will not initialize or some other kind of crash.
            case Level::Type::FATAL: return "FATAL"; break; //This Level implies that a program has failed so bad it can cause injury to itself or others.
            default: return LevelString(Level::Type::UNKNOWN); break;
        }
    }
};

class ArmDisarm{
    public:
    enum class Type{
        UNKNOWN=0,
        ARMED=1,
        DISARMED_CANNOTARM=2,
        DISARMED=3,
        DISARMING=4,
        ARMING=5,
        END_OF_LIST=6
    };
    static std::string ArmDisarmString(ArmDisarm::Type v) {
        switch(v)
        {
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

class Command{
    public:
    enum class Type{
        UNKNOWN=0,
        NONE=1,
        RUN_DIAGNOSTIC=2,
        SEARCHFOR_RECHARGE_FACILITY=3,
        ACQUIRE_TARGET=4,
        ARM=5,
        DISARM=6,
        CONFIGURE=7,
        RUN=8,
        STOPMOVEMENT=9,
        DRIVE=10,
        RESET=11,
        SETLOGLEVEL=12,
        TASKCONTROL=13,
        CALIBRATION=14,
        GENERATE_SNAPSHOT=15,
        WAIT=16,
        END_OF_LIST=17
    };
    static std::string CommandString(Command::Type v) {
        switch(v)
        {
            case Command::Type::UNKNOWN: return "UNKNOWN"; break;
            case Command::Type::NONE: return "NONE"; break;
            case Command::Type::RUN_DIAGNOSTIC: return "RUN_DIAGNOSTIC"; break;
            case Command::Type::SEARCHFOR_RECHARGE_FACILITY: return "SEARCHFOR_RECHARGE_FACILITY"; break;
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
class Node
{
    public:
    enum class State{
        UNKNOWN=0,
        START=1,
        INITIALIZING=2,
        INITIALIZED=3,
        LOADING=4,
        RUNNING=5,
        PAUSED=6,
        RESET=7,
        NODATA=8,
        FINISHED=9,
        CRASHED=10,
        END_OF_LIST=11
    };
    static std::string NodeStateString(Node::State v)
    {
        switch(v)
        {
            case Node::State::UNKNOWN: return "UNKNOWN"; break;
            case Node::State::START: return "START"; break;
            case Node::State::INITIALIZING: return "INITIALIZING"; break;
            case Node::State::INITIALIZED: return "INITIALIZED"; break;
            case Node::State::LOADING: return "LOADING"; break;
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
#endif // EROSDEFINITIONS_H
