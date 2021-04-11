/*! \file MasterNodeProcess.h
 */
#ifndef MasterNodeProcess_H
#define MasterNodeProcess_H
#include <eros/BaseNodeProcess.h>
namespace eros_nodes {
/*! \class MasterNodeProcess MasterNodeProcess.h "MasterNodeProcess.h"
 *  \brief */
class MasterNodeProcess : public eros::BaseNodeProcess
{
   public:
    MasterNodeProcess();
    ~MasterNodeProcess();
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    eros::Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<eros::Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
}  // namespace eros_nodes
#endif  // MasterNodeProcess_H
