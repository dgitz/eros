/*! \file ArmedStateManager.h
 */
#pragma once
#include <eros/armed_state.h>
#include <eros/command.h>
#include <eros_diagnostic/Diagnostic.h>
#include <eros_diagnostic/DiagnosticUtility.h>

namespace eros {
/*! \class ArmedStateManager
    \brief ArmedStateManager class is responsible for managing the robot's Armed State.
*/
class ArmedStateManager
{
   public:
    ArmedStateManager(std::string device_name,
                      std::string node_name,
                      System::MainSystem system,
                      System::SubSystem sub_system);
    ~ArmedStateManager();
    // Constants

    // Enums

    // Structs

    // Initialization Functions
    bool reset();

    // Update Functions
    eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions
    eros_diagnostic::Diagnostic get_current_diagnostic() {
        return current_diagnostic;
    }
    ArmDisarm::State get_armed_state() {
        return current_armed_state;
    }
    armed_state get_armed_state_msg() {
        eros::armed_state msg;
        msg.armed_state = (uint8_t)current_armed_state.state;
        return msg;
    }

    // Utility Functions

    // Support Functions

    // Message Functions
    bool new_command(eros::command cmd);
    bool new_all_ready_to_arm(bool data);

    // Destructors
    void cleanup() {
        return;
    }

    // Printing Functions
    std::string pretty();

   private:
    eros_diagnostic::Diagnostic current_diagnostic;
    ArmDisarm::State current_armed_state;
};
}  // namespace eros