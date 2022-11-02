#ifndef BASEWINDOW_H
#define BASEWINDOW_H
#include <eros/SystemMonitor/DataStructures.h>
#include <eros/eROS_Definitions.h>

#include "IWindow.h"
namespace eros {
class BaseWindow : public IWindow
{
   public:
    BaseWindow(eros::Logger* logger, WindowType windowType)
        : logger(logger), windowType(windowType) {
    }
    virtual ~BaseWindow() {
    }
    WindowType getWindowType() {
        return windowType;
    }
    bool update(double currentTime_s);
    static Color convertLevelToColor(Level::Type level);
    static Color convertNodeStateToColor(Node::State state);
    void setActualWindowSize(ScreenCoordinatePixel size) {
        actualWindowSize = size;
    }

   protected:
    eros::Logger* logger;
    WindowType windowType;
    ScreenCoordinatePixel actualWindowSize;
};
}  // namespace eros
#endif