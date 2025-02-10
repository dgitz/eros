/*! \file MasterNodeProcess.h
 */
#ifndef MasterNodeProcess_H
#define MasterNodeProcess_H
#include <eros/BaseNodeProcess.h>
namespace eros_nodes {
/*! \class MasterNodeProcess MasterNodeProcess.h "MasterNodeProcess.h"
 *  \brief The process utility for the Master Node. */
class MasterNodeProcess : public eros::BaseNodeProcess
{
   public:
    // Constants

    // Enums

    // Structs

    // Initialization Functions

    // Update Functions

    // Attribute Functions

    // Utility Functions

    // Support Functions

    // Message Functions

    // Destructors

    // Printing Functions

    // TODO
    MasterNodeProcess();
    ~MasterNodeProcess();
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
}  // namespace eros_nodes
#endif  // MasterNodeProcess_H
