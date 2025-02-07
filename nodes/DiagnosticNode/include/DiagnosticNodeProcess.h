/*! \file DiagnosticNodeProcess.h
 */
#ifndef DiagnosticNodeProcess_H
#define DiagnosticNodeProcess_H
#include <eros/BaseNodeProcess.h>
namespace eros_nodes {
/*! \class DiagnosticNodeProcess DiagnosticNodeProcess.h "DiagnosticNodeProcess.h"
 *  \brief The process utility for the DiagnosticNode. */
class DiagnosticNodeProcess : public eros::BaseNodeProcess
{
   public:
    /*! \struct DiagnosticTypeAggregate
        \brief DiagnosticTypeAggregate Container
        Container to aggregate Diagnostic Information.
    */
    struct DiagnosticTypeAggregate {
        DiagnosticTypeAggregate() : update_count(0) {
        }
        eros::Diagnostic::DiagnosticDefinition worst_diag;
        std::string worst_diag_key;
        uint64_t update_count;
        std::map<std::string, eros::Diagnostic::DiagnosticDefinition> diag_list;
    };
    DiagnosticNodeProcess();
    ~DiagnosticNodeProcess();
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    eros::Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    bool new_external_diagnostic(eros::Diagnostic::DiagnosticDefinition diag);
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<eros::Diagnostic::DiagnosticDefinition> check_programvariables();
    eros::Diagnostic::DiagnosticDefinition update_topiclist(
        std::vector<std::string>& new_diagnostic_topics_to_subscribe);
    eros::Diagnostic::DiagnosticDefinition get_worst_diagnostic(
        eros::Diagnostic::DiagnosticType type);
    void cleanup() {
        base_cleanup();
        return;
    }
    std::string pretty();

   private:
    std::string build_key(eros::Diagnostic::DiagnosticDefinition);
    std::vector<std::string> diagnostic_topics_already_monitoring;
    std::map<eros::Diagnostic::DiagnosticType, DiagnosticTypeAggregate> diagnostic_aggregator;
};
}  // namespace eros_nodes
#endif  // DiagnosticNodeProcess_H