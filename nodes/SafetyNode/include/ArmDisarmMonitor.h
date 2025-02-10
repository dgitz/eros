#pragma once
#include <eros/ready_to_arm.h>

#include <string>

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
}  // namespace eros_nodes