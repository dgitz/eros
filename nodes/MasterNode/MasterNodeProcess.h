/*! \file MasterNodeProcess.h
 */
#ifndef MasterNodeProcess_H
#define MasterNodeProcess_H
#include <eros/BaseNodeProcess.h>
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
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

    Architecture::Type read_device_architecture();

   private:
};
#endif  // MasterNodeProcess_H
