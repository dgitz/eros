/*! \file SnapshotProcess.h
 */
#ifndef SnapshotProcess_H
#define SnapshotProcess_H
#include <eros/BaseNodeProcess.h>
#include <tinyxml.h>

#include <boost/thread.hpp>
#include <boost/thread/scoped_thread.hpp>
#include <fstream>
namespace eros_nodes {
/*! \class SnapshotProcess SnapshotProcess.h "SnapshotProcess.h"
 *  \brief */
class SnapshotProcess : public eros::BaseNodeProcess
{
   public:
    SnapshotProcess()
        : nodeHandle(nullptr),
          robot_namespace(""),
          mode(Mode::UNKNOWN),
          architecture(eros::Architecture::Type::UNKNOWN),
          devicesnapshot_state(SnapshotState::NOTRUNNING),
          systemsnapshot_state(SnapshotState::NOTRUNNING),
          snapshot_progress_percent(0.0),
          holdcomplete_timer(0.0){};
    ~SnapshotProcess();
    const double HOLDCOMPLETE_TIME = 5.0;
    enum class Mode { UNKNOWN = 0, MASTER = 1, SLAVE = 2, END_OF_LIST = 3 };

    enum class SnapshotState {
        UNKNOWN = 0,
        NOTRUNNING = 1,
        STARTED = 2,
        RUNNING = 3,
        READY = 4,
        COMPLETE = 5,
        INCOMPLETE = 6,
        END_OF_LIST = 7
    };
    static std::string SnapshotStateString(SnapshotState v) {
        switch (v) {
            case SnapshotState::UNKNOWN: return "UNKNOWN";
            case SnapshotState::NOTRUNNING: return "NOT RUNNING";
            case SnapshotState::STARTED: return "STARTED";
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
    struct ExecCommand {
        std::string command;
        std::string output_file;
    };
    struct SlaveDevice {
        SlaveDevice(std::string _name, std::string _id)
            : name(_name),
              id(_id),
              device_snapshot_generated(false),
              timer(0.0),
              devicesnapshot_path("") {
        }
        std::string name;
        std::string id;
        bool device_snapshot_generated;
        bool device_snapshot_processed;
        double timer;
        std::string devicesnapshot_path;
    };
    struct SnapshotConfig {
        std::string stage_directory;

        std::vector<SlaveDevice> snapshot_devices;
        std::vector<std::string> folders;
        std::vector<std::string> files;
        std::vector<ExecCommand> commands;
        std::vector<std::string> scripts;
        std::string systemsnapshot_path;
        std::string device_snapshot_path;
        std::string bagfile_directory;
        std::string active_device_snapshot_completepath;
    };
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    Mode get_mode() {
        return mode;
    }
    void set_mode(Mode v) {
        mode = v;
    }
    void set_architecture(eros::Architecture::Type v) {
        architecture = v;
    }
    SnapshotConfig get_snapshot_config() {
        return snapshot_config;
    }
    SnapshotState get_devicesnapshot_state() {
        return devicesnapshot_state;
    }
    void set_devicesnapshot_state(SnapshotState v) {
        devicesnapshot_state = v;
    }
    SnapshotState get_systemsnapshot_state() {
        return systemsnapshot_state;
    }
    void set_systemsnapshot_state(SnapshotState v) {
        systemsnapshot_state = v;
    }
    double get_snapshotprogress_percentage() {
        return snapshot_progress_percent;
    }
    std::vector<eros::Diagnostic::DiagnosticDefinition> clear_snapshots();
    bool set_nodeHandle(ros::NodeHandle* nh, std::string _robot_namespace) {
        nodeHandle = nh;
        robot_namespace = _robot_namespace;
        return true;
    }
    eros::Diagnostic::DiagnosticDefinition load_config(
        std::string file_path, std::vector<std::string> override_devicenames);
    void reset();
    eros::Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command t_msg);
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandstatemsg(
        eros::command_state t_msg);
    std::vector<eros::Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    std::string pretty();
    std::vector<eros::Diagnostic::DiagnosticDefinition> createnew_snapshot();

   private:
    int count_files_indirectory(std::string directory, std::string filter);
    ros::NodeHandle* nodeHandle;
    std::string robot_namespace;
    Mode mode;
    eros::Architecture::Type architecture;
    SnapshotState devicesnapshot_state;
    SnapshotState systemsnapshot_state;
    SnapshotConfig snapshot_config;
    double snapshot_progress_percent;
    double holdcomplete_timer;
};
}  // namespace eros_nodes
#endif  // SnapshotProcess_H
