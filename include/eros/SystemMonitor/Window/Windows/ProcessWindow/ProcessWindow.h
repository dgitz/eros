#ifndef PROCESSWINDOW_H
#define PROCESSWINDOW_H
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/WindowTable.h>
#include <eros/heartbeat.h>
#include <eros/resource.h>

#include <boost/range/algorithm/count.hpp>

#include "ProcessManager.h"
namespace eros {
class ProcessWindow : public WindowTable
{
   public:
    static constexpr double COMMTIMEOUT_S = 5.0;
    ProcessWindow(eros::Logger* logger)
        : WindowTable(logger, IWindow::WindowType::PROCESS), processManager(logger, COMMTIMEOUT_S) {
        std::vector<ColumnLabel> columnLabels;
        columnLabels.push_back(ColumnLabel(" ? ", 3));
        columnLabels.push_back(ColumnLabel(" ID ", 4));
        columnLabels.push_back(ColumnLabel(" Host ", 20));
        columnLabels.push_back(ColumnLabel(" Node Name ", 24));
        columnLabels.push_back(ColumnLabel(" Status ", 14));
        columnLabels.push_back(ColumnLabel(" Restarts ", 10));
        columnLabels.push_back(ColumnLabel(" PID ", 8));
        columnLabels.push_back(ColumnLabel(" CPU(%) ", 8));
        columnLabels.push_back(ColumnLabel(" RAM(%) ", 8));
        columnLabels.push_back(ColumnLabel(" Rx ", 5));
        setColumnLabels(columnLabels);
    }
    virtual ~ProcessWindow() {
    }
    WindowSize getWindowSize();
    std::vector<std::shared_ptr<IRecord>> getRecords();
    bool new_heartbeat(eros::heartbeat msg);
    bool new_resource(eros::resource msg);
    bool new_nodeAlive(std::string hostName, std::string nodeName, double currentTime_s);
    bool keyPressed(KeyMap key);
    bool update(double currentTime_s);
    // Not allowed to set records independently.
    bool setRecords(std::vector<std::shared_ptr<IRecord>> records) {
        (void)records;
        return false;
    }
    static std::string smallifyROSName(std::string v);

   private:
    ProcessManager processManager;
};
}  // namespace eros
#endif