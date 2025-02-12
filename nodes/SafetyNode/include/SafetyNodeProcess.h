/*! \file SafetyNodeProcess.h
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <ros/ros.h>

namespace eros_nodes {
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

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    bool new_message_readytoarm(std::string name, eros::ready_to_arm ready_to_arm);

    // Destructors
    void cleanup() {
        base_cleanup();
        return;
    }

    // Printing Functions

   private:
};
}  // namespace eros_nodes
