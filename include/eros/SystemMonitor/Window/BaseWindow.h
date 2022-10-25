#ifndef BASEWINDOW_H
#define BASEWINDOW_H
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

   protected:
    eros::Logger* logger;
    WindowType windowType;
};
}  // namespace eros
#endif