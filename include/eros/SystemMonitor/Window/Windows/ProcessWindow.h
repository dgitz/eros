#ifndef PROCESSWINDOW_H
#define PROCESSWINDOW_H
#include <eros/SystemMonitor/Window/WindowTable.h>
#include <eros/heartbeat.h>
namespace eros {
class ProcessWindow : public WindowTable
{
   public:
    ProcessWindow(): updateCounter(0) {
    }
    WindowSize getWindowSize();
    std::vector<RenderData> getData();
    bool new_heartbeat(eros::heartbeat msg);

   private:
   uint64_t updateCounter;
};
}  // namespace eros
#endif