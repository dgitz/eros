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
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    std::string pretty() override;

   private:
};
#endif  // SampleNodeProcess_H
