/*! \file SampleNodeProcess.h
 */
#ifndef SAMPLENODEPROCESS_H
#define SAMPLENODEPROCESS_H
#include <eros/BaseNodeProcess.h>
/*! \class SampleNodeProcess SampleNodeProcess.h "SampleNodeProcess.h"
 *  \brief This is a SampleNodeProcess class illustrating usage of the EROS BaseNodeProcess. */
class SampleNodeProcess : public BaseNodeProcess
{
   public:
    SampleNodeProcess();
    ~SampleNodeProcess();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();

   private:
};
#endif