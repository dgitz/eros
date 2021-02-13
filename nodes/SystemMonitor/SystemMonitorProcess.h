/*! \file SystemMonitorProcess.h
 */
#ifndef SYSTEMMONITORPROCESS_h
#define SYSTEMMONITORPROCESS_h
#include <eros/BaseNodeProcess.h>
/*! \class SystemMonitorProcess SystemMonitorProcess.h "SystemMonitorProcess.h"
 *  \brief  */
class SystemMonitorProcess : public BaseNodeProcess
{
   public:
    SystemMonitorProcess();
    ~SystemMonitorProcess();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();

   private:
};
#endif  // SYSTEMMONITORPROCESS_h