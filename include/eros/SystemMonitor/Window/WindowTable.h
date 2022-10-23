#ifndef WINDOWTABLE_H
#define WINDOWTABLE_H
#include "BaseWindow.h"
namespace eros {
class WindowTable : public BaseWindow
{
   public:
    WindowTable(eros::Logger* logger) : BaseWindow(logger) {
    }
    virtual ~WindowTable() {
    }
};
}  // namespace eros
#endif