#ifndef NODEDIAGNOSTICSWINDOW_H
#define NODEDIAGNOSTICSWINDOW_H
#include <eros/SystemMonitor/Window/WindowTable.h>
namespace eros {
class NodeDiagnosticsWindow : public WindowTable
{
   public:
    NodeDiagnosticsWindow() {
    }
    WindowSize getWindowSize();
    std::vector<RenderData> getData();

   private:
};
}  // namespace eros
#endif