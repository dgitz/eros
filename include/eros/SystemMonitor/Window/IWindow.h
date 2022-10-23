#ifndef IWINDOW_H
#define IWINDOW_H
#include <eros/SystemMonitor/DataStructures.h>

#include <string>
#include <vector>
namespace eros {
class IWindow
{
   public:
    enum class WindowType {
        UNKNOWN = 0,
        PROCESS = 1,
        NODEDIAGNOSTICS = 2,
        DEVICE = 3,
        HEADER = 4,
        INFO = 5,
        END_OF_LIST = 6
    };
    virtual WindowSize getWindowSize() = 0;
    virtual std::vector<RenderData> getData() = 0;

   private:
};
}  // namespace eros
#endif