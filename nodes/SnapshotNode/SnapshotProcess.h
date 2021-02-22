/*! \file SnapshotProcess.h
 */
#ifndef SnapshotProcess_H
#define SnapshotProcess_H
#include <eros/BaseNodeProcess.h>
#include <tinyxml.h>
/*! \class SnapshotProcess SnapshotProcess.h "SnapshotProcess.h"
 *  \brief */
class SnapshotProcess : public BaseNodeProcess
{
   public:
    SnapshotProcess();
    ~SnapshotProcess();
    enum class Mode { UNKNOWN = 0, MASTER = 1, SLAVE = 2, END_OF_LIST = 3 };
    static Mode ModeType(std::string v) {
        if (v == "MASTER") {
            return Mode::MASTER;
        }
        else if (v == "SLAVE") {
            return Mode::SLAVE;
        }
        else {
            return Mode::UNKNOWN;
        }
    }
    static std::string ModeString(Mode v) {
        switch (v) {
            case Mode::UNKNOWN: return "UNKNOWN"; break;
            case Mode::MASTER: return "MASTER";
            case Mode::SLAVE: return "SLAVE";
            default: return ModeString(Mode::UNKNOWN); break;
        }
    }
    Diagnostic::DiagnosticDefinition finish_initialization();
    Mode get_mode() {
        return mode;
    }
    void set_mode(Mode v) {
        mode = v;
    }
    void set_architecture(Architecture::Type v) {
        architecture = v;
    }
    Diagnostic::DiagnosticDefinition load_config(std::string file_path);
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
    Mode mode;
    Architecture::Type architecture;
};
#endif  // SnapshotProcess_H
