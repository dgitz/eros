/*! \file SampleNodeProcess.h
 */
#ifndef SampleNodeProcess_H
#define SampleNodeProcess_H
#include <eros/BaseNodeProcess.h>
/*! \class SampleNodeProcess SampleNodeProcess.h "SampleNodeProcess.h"
 *  \brief */
class SampleNodeProcess : public eros::BaseNodeProcess
{
   public:
    SampleNodeProcess();
    ~SampleNodeProcess();
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    eros::Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandmsg(
        eros::command msg);
    std::vector<eros::Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
#endif // SampleNodeProcess_H
