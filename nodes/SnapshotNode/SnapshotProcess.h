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
    enum class SnapshotState {
        UNKNOWN = 0,
        NOTRUNNING = 1,
        RUNNING = 2,
        READY = 3,
        COMPLETE = 4,
        INCOMPLETE = 5,
        END_OF_LIST = 6
    };
    static std::string SnapshotStateString(SnapshotState v) {
        switch (v) {
            case SnapshotState::UNKNOWN: return "UNKNOWN";
            case SnapshotState::NOTRUNNING: return "NOT RUNNING";
            case SnapshotState::RUNNING: return "RUNNING";
            case SnapshotState::READY: return "READY";
            case SnapshotState::COMPLETE: return "COMPLETE";
            case SnapshotState::INCOMPLETE: return "INCOMPLETE";
            default: return SnapshotStateString(SnapshotState::UNKNOWN);
        }
    };

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
    struct Command {
        std::string command;
        std::string output_file;
    };
    struct SnapshotConfig {
        std::string stage_directory;
        std::vector<std::string> folders;
        std::vector<std::string> files;
        std::vector<Command> commands;
        std::vector<std::string> scripts;
        std::string device_snapshot_path;
    };
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
    SnapshotConfig get_snapshot_config() {
        return snapshot_config;
    }
    SnapshotState get_devicesnapshot_state() {
        return devicesnapshot_state;
    }
    SnapshotState get_systemsnapshot_state() {
        return systemsnapshot_state;
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
    std::string pretty();
    std::vector<Diagnostic::DiagnosticDefinition> createnew_snapshot();

   private:
    int count_files_indirectory(std::string directory, std::string filter);
    Mode mode;
    Architecture::Type architecture;
    SnapshotState devicesnapshot_state;
    SnapshotState systemsnapshot_state;
    SnapshotConfig snapshot_config;
};
#endif  // SnapshotProcess_H
