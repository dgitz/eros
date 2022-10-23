#ifndef HEADERWINDOW_H
#define HEADERWINDOW_H
#include <eros/SystemMonitor/Window/WindowText.h>
namespace eros {
class HeaderWindow : public WindowText
{
   public:
    HeaderWindow() {
    }
    WindowSize getWindowSize();
    std::vector<RenderData> getData();

   private:
};
}  // namespace eros
#endif