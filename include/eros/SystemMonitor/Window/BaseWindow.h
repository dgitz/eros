#ifndef BASEWINDOW_H
#define BASEWINDOW_H
#include "IWindow.h"
namespace eros {
class BaseWindow : public IWindow
{
   public:
    BaseWindow(eros::Logger* logger) : logger(logger) {
    }
    virtual ~BaseWindow() {
    }

   protected:
    eros::Logger* logger;
};
}  // namespace eros
#endif