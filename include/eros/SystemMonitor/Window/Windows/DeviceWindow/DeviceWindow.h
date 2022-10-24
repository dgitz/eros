#ifndef DEVICEWINDOW_H
#define DEVICEWINDOW_H
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/WindowTable.h>
namespace eros {
class DeviceWindow : public WindowTable
{
   public:
    DeviceWindow(eros::Logger* logger) : WindowTable(logger) {
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

   private:
};
}  // namespace eros
#endif