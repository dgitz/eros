#ifndef WINDOWTEXT_H
#define WINDOWTEXT_H
#include "BaseWindow.h"
namespace eros {
class WindowText : public BaseWindow
{
   public:
    WindowText(eros::Logger* logger, WindowType windowType) : BaseWindow(logger, windowType) {
    }
    virtual ~WindowText() {
    }
};
}  // namespace eros
#endif