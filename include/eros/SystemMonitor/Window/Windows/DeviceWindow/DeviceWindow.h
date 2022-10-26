#ifndef DEVICEWINDOW_H
#define DEVICEWINDOW_H
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/WindowTable.h>
#include <eros/heartbeat.h>
#include <eros/loadfactor.h>
#include <eros/resource.h>
namespace eros {
class DeviceWindow : public WindowTable
{
   public:
    DeviceWindow(eros::Logger* logger) : WindowTable(logger, IWindow::WindowType::DEVICE) {
        std::vector<ColumnLabel> columnLabels;
        columnLabels.push_back(ColumnLabel(" ID ", 4));
        columnLabels.push_back(ColumnLabel(" Device ", 14));
        columnLabels.push_back(ColumnLabel(" CPU Av ", 8));
        columnLabels.push_back(ColumnLabel(" RAM Av ", 8));
        columnLabels.push_back(ColumnLabel(" Disk Av ", 9));
        columnLabels.push_back(ColumnLabel(" Load Factor ", 22));
        columnLabels.push_back(ColumnLabel(" Rx ", 5));
        setColumnLabels(columnLabels);
    }
    virtual ~DeviceWindow() {
    }
    WindowSize getWindowSize();
    std::vector<std::shared_ptr<IRecord>> getRecords();
    // Not allowed to set records independently.
    bool setRecords(std::vector<std::shared_ptr<IRecord>> records) {
        (void)records;
        return false;
    }
    bool keyPressed(KeyMap key);
    bool new_loadfactor(eros::loadfactor msg) {
        return false;
    }
    bool new_resourceavailable(eros::resource msg) {
        return false;
    }

   private:
};
}  // namespace eros
#endif