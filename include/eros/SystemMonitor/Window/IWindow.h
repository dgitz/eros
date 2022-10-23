#ifndef IWINDOW_H
#define IWINDOW_H
#include <eros/Logger.h>
#include <eros/SystemMonitor/DataStructures.h>
#include <eros/SystemMonitor/Record/IRecord.h>

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
        STATUS = 4,
        INFO = 5,
        END_OF_LIST = 6
    };
    virtual ~IWindow() {
    }
    virtual WindowSize getWindowSize() = 0;
    virtual std::vector<IRecord*> getRecords() = 0;
    virtual bool keyPressed(KeyMap key) = 0;

   private:
};
}  // namespace eros
#endif