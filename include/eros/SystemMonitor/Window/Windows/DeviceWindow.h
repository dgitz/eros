#ifndef DEVICEWINDOW_H
#define DEVICEWINDOW_H
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
    std::vector<RenderData> getData();
    bool keyPressed(KeyMap key);

   private:
};
}  // namespace eros
#endif