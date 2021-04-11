/*! \file MasterNodeProcess.h
 */
#ifndef MasterNodeProcess_H
#define MasterNodeProcess_H
#include <eros/BaseNodeProcess.h>
using namespace eros;
namespace eros_nodes {
/*! \class MasterNodeProcess MasterNodeProcess.h "MasterNodeProcess.h"
 *  \brief */
class MasterNodeProcess : public BaseNodeProcess
{
   public:
    MasterNodeProcess();
    ~MasterNodeProcess();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
}  // namespace eros_nodes
#endif  // MasterNodeProcess_H
