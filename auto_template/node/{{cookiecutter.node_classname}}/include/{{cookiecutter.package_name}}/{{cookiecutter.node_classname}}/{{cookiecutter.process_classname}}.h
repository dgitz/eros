/*! \file {{cookiecutter.process_classname}}.h
 */
#ifndef {{cookiecutter.process_classname}}_H
#define {{cookiecutter.process_classname}}_H
#include <eros/BaseNodeProcess.h>
/*! \class {{cookiecutter.process_classname}} {{cookiecutter.process_classname}}.h "{{cookiecutter.process_classname}}.h"
 *  \brief */
class {{cookiecutter.process_classname}} : public eros::BaseNodeProcess
{
   public:
    {{cookiecutter.process_classname}}();
    ~{{cookiecutter.process_classname}}();
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(
        eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
#endif // {{cookiecutter.process_classname}}_H
