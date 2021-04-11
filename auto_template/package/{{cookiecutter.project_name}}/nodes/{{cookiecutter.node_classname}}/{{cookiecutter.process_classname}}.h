/*! \file {{cookiecutter.process_classname}}.h
 */
#ifndef {{cookiecutter.process_classname}}_H
#define {{cookiecutter.process_classname}}_H
#include <eros/BaseNodeProcess.h>
/*! \class {{cookiecutter.process_classname}} {{cookiecutter.process_classname}}.h "{{cookiecutter.process_classname}}.h"
 *  \brief */
class {{cookiecutter.process_classname}} : public BaseNodeProcess
{
   public:
    {{cookiecutter.process_classname}}();
    ~{{cookiecutter.process_classname}}();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        eros::command msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
#endif // {{cookiecutter.process_classname}}_H
