/*! \file DataLoggerProcess.h
 */
#ifndef DataLoggerProcess_H
#define DataLoggerProcess_H
#include <eros/BaseNodeProcess.h>
#include <sys/stat.h>
namespace eros_nodes {
/*! \class DataLoggerProcess DataLoggerProcess.h "DataLoggerProcess.h"
 *  \brief
    The process utility for the DataLogger
 */
class DataLoggerProcess : public eros::BaseNodeProcess
{
   public:
    DataLoggerProcess();
    ~DataLoggerProcess();
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    void set_logfileduration(double v) {
        logfile_duration = v;
    }
    double get_logfile_duration() {
        return logfile_duration;
    }
    bool set_logdirectory(std::string v) {
        log_directory = v;
        log_directory = sanitize_path(log_directory);
        log_directory_available = false;
        logging_enabled = false;
        struct stat status;
        if (stat(log_directory.c_str(), &status) == 0) {
            log_directory_available = true;
            logging_enabled = true;
        }

        return log_directory_available;
    }
    bool is_logging_enabled() {
        return logging_enabled;
    }
    std::string get_logdirectory() {
        return log_directory;
    }
    bool getSnapshotMode() {
        return snapshot_mode;
    }
    void setSnapshotMode(bool v) {
        snapshot_mode = v;
    }

   private:
    std::string log_directory;
    bool log_directory_available;
    double logfile_duration;
    bool logging_enabled;
    bool snapshot_mode;
};
}  // namespace eros_nodes
#endif  // DataLoggerProcess_H