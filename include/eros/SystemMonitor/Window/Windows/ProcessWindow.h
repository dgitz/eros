#ifndef PROCESSWINDOW_H
#define PROCESSWINDOW_H
#include <eros/SystemMonitor/Window/WindowTable.h>
#include <eros/heartbeat.h>
namespace eros {
class ProcessWindow : public WindowTable
{
   public:
    ProcessWindow(eros::Logger* logger) : WindowTable(logger), updateCounter(0) {
    }
    virtual ~ProcessWindow() {
    }
    WindowSize getWindowSize();
    std::vector<RenderData> getData();
    bool new_heartbeat(eros::heartbeat msg);
    bool keyPressed(KeyMap key);

   private:
    uint64_t updateCounter;
};
}  // namespace eros
#endif