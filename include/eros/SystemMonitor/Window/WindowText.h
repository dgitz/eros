#ifndef WINDOWTEXT_H
#define WINDOWTEXT_H
#include "BaseWindow.h"
namespace eros {
class WindowText : public BaseWindow
{
   public:
    WindowText(eros::Logger* logger) : BaseWindow(logger) {
    }
    virtual ~WindowText() {
    }
};
}  // namespace eros
#endif