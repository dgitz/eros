#ifndef WINDOWTABLE_H
#define WINDOWTABLE_H
#include "BaseWindow.h"
namespace eros {
class WindowTable : public BaseWindow
{
   public:
    struct ColumnLabel {
        ColumnLabel(std::string label, uint16_t minWidth) : label(label), minWidth(minWidth) {
        }
        std::string label;
        uint16_t minWidth;
    };
    WindowTable(eros::Logger* logger, WindowType windowType) : BaseWindow(logger, windowType) {
    }
    virtual ~WindowTable() {
    }
    void setColumnLabels(std::vector<ColumnLabel> columnLabels);
    std::vector<ColumnLabel> getColumnLabels() {
        return columnLabels;
    }

   private:
    std::vector<ColumnLabel> columnLabels;
};
}  // namespace eros
#endif