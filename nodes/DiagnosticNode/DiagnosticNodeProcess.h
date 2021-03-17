/*! \file DiagnosticNodeProcess.h
 */
#ifndef DiagnosticNodeProcess_H
#define DiagnosticNodeProcess_H
#include <eros/BaseNodeProcess.h>
/*! \class DiagnosticNodeProcess DiagnosticNodeProcess.h "DiagnosticNodeProcess.h"
 *  \brief */
class DiagnosticNodeProcess : public BaseNodeProcess
{
   public:
    struct DiagnosticTypeAggregate {
        DiagnosticTypeAggregate() : update_count(0) {
        }
        Diagnostic::DiagnosticDefinition worst_diag;
        std::string worst_diag_key;
        uint64_t update_count;
        std::map<std::string, Diagnostic::DiagnosticDefinition> diag_list;
    };
    DiagnosticNodeProcess();
    ~DiagnosticNodeProcess();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    bool new_external_diagnostic(Diagnostic::DiagnosticDefinition diag);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    Diagnostic::DiagnosticDefinition update_topiclist(
        std::vector<std::string>& new_diagnostic_topics_to_subscribe);
    Diagnostic::DiagnosticDefinition get_worst_diagnostic(Diagnostic::DiagnosticType type);
    void cleanup() {
        base_cleanup();
        return;
    }
    std::string pretty();

   private:
    std::string build_key(Diagnostic::DiagnosticDefinition);
    std::vector<std::string> diagnostic_topics_already_monitoring;
    std::map<Diagnostic::DiagnosticType, DiagnosticTypeAggregate> diagnostic_aggregator;
};
#endif  // DiagnosticNodeProcess_H
