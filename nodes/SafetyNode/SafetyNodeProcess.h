/*! \file SafetyNodeProcess.h
 */
#ifndef SafetyNodeProcess_H
#define SafetyNodeProcess_H
#include <eros/BaseNodeProcess.h>
/*! \class SafetyNodeProcess SafetyNodeProcess.h "SafetyNodeProcess.h"
 *  \brief */
class SafetyNodeProcess : public BaseNodeProcess
{
   public:
    SafetyNodeProcess();
    ~SafetyNodeProcess();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    ArmDisarm::State get_armed_state() {
        return armed_state;
    }

   private:
    ArmDisarm::State armed_state;
};
#endif  // SafetyNodeProcess_H
