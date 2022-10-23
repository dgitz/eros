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
    std::vector<IRecord*> getRecords();
    bool keyPressed(KeyMap key);

   private:
};
}  // namespace eros
#endif