#ifndef WINDOWTABLE_H
#define WINDOWTABLE_H
#include "BaseWindow.h"
namespace eros {
class WindowTable : public BaseWindow
{
   public:
    std::string getData();
    WindowSize getWindowSize();
};
}  // namespace eros
#endif