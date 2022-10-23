#ifndef INFOWINDOW_H
#define INFOWINDOW_H
#include <eros/SystemMonitor/Window/WindowText.h>
namespace eros {
class InfoWindow : public WindowText
{
   public:
    InfoWindow() {
    }
    WindowSize getWindowSize();
    std::vector<RenderData> getData();

   private:
};
}  // namespace eros
#endif