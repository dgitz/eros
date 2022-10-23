#ifndef STATUSWINDOW_H
#define STATUSWINDOW_H
#include <eros/SystemMonitor/Window/WindowText.h>
namespace eros {
class StatusWindow : public WindowText
{
   public:
    StatusWindow(eros::Logger* logger) : WindowText(logger) {
    }
    virtual ~StatusWindow() {
    }
    WindowSize getWindowSize();
    std::vector<RenderData> getData();
    bool keyPressed(KeyMap key);

   private:
};
}  // namespace eros
#endif