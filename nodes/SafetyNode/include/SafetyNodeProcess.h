/*! \file SafetyNodeProcess.h
 */
#ifndef SafetyNodeProcess_H
#define SafetyNodeProcess_H
#include <eros/BaseNodeProcess.h>
#include <ros/ros.h>
namespace eros_nodes {
/*! \class ArmDisarmMonitor
    \brief ArmDisarmMonitor class
*/
class ArmDisarmMonitor
{
   public:
    static constexpr double READYTOARM_TIMEOUT = 5.0f;
    enum class Type {
        UNKNOWN = 0,    /*!< Uninitialized value. */
        DEFAULT = 1,    /*!< Default Type, uses a ready_to_arm topic which has a bool flag and a
                           diagnostic message */
        SIMPLE = 2,     /*!< Simple Type, just a Bool message.  A diagnostic will be created if this
                           trigger is False (not ready to arm) */
        END_OF_LIST = 3 /*!< Last item of list. Used for Range Checks. */
    };
    static Type TypeEnum(std::string v) {
        if (v == "DEFAULT") {
            return Type::DEFAULT;
        }
        else if (v == "SIMPLE") {
            return Type::SIMPLE;
        }
        else {
            return Type::UNKNOWN;
        }
    }
    ArmDisarmMonitor(std::string _name, Type _type);

    ~ArmDisarmMonitor();

    std::string name;
    Type type;
    eros::ready_to_arm ready_to_arm;
    uint64_t update_count;
    double last_delta_update_time;

   private:
};
/*! \class SafetyNodeProcess SafetyNodeProcess.h "SafetyNodeProcess.h"
 *  \brief The process utility for the Safety Node. */
class SafetyNodeProcess : public eros::BaseNodeProcess
{
   public:
    SafetyNodeProcess();
    ~SafetyNodeProcess();
    // Constants

    // Enums

    // Structs

    // Initialization Functions
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    bool initialize_readytoarm_monitors(std::vector<std::string> topics,
                                        std::vector<ArmDisarmMonitor::Type> types);
    bool init_ros(boost::shared_ptr<ros::NodeHandle> _n);

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions
    std::vector<std::string> get_cannotarm_reasons() {
        return cannotarm_reasons;
    }
    eros::ArmDisarm::State get_armed_state() {
        return armed_state;
    }

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    bool new_message_readytoarm(std::string name, eros::ready_to_arm ready_to_arm);
    bool new_message_readytoarm(std::string name, bool v);
    void ReadyToArmDefaultCallback(const eros::ready_to_arm::ConstPtr &msg,
                                   const std::string &topic_name);
    void ReadyToArmSimpleCallback(const std_msgs::Bool::ConstPtr &msg,
                                  const std::string &topic_name);

    // Destructors
    void cleanup() {
        base_cleanup();
        return;
    }

    // Printing Functions

   private:
    boost::shared_ptr<ros::NodeHandle> nodeHandle;
    eros::ArmDisarm::State armed_state;
    std::map<std::string, ArmDisarmMonitor> ready_to_arm_monitors;
    std::vector<std::string> cannotarm_reasons;
    std::vector<ros::Subscriber> arm_monitor_subs;
};
}  // namespace eros_nodes
#endif  // SafetyNodeProcess_H
