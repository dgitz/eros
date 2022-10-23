#ifndef DEVICEWINDOW_H
#define DEVICEWINDOW_H
#include <eros/SystemMonitor/Window/WindowTable.h>
namespace eros {
class DeviceWindow : public WindowTable
{
   public:
    DeviceWindow() {
    }
    WindowSize getWindowSize();
    std::vector<RenderData> getData();

   private:
};
}  // namespace eros
#endif