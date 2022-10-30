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
    WindowTable(eros::Logger* logger, WindowType windowType)
        : BaseWindow(logger, windowType), recordCount(0), selectedRecordIndex(0) {
    }
    virtual ~WindowTable() {
    }
    void setColumnLabels(std::vector<ColumnLabel> columnLabels);
    std::vector<ColumnLabel> getColumnLabels() {
        return columnLabels;
    }
    bool keyPressed(KeyMap key);
    uint16_t getSelectedRecordIndex() {
        return selectedRecordIndex;
    }

   protected:
    uint64_t recordCount;
    uint16_t selectedRecordIndex;

   private:
    bool incrementSelectedRecord();
    bool decrementSelectedRecord();
    std::vector<ColumnLabel> columnLabels;
};
}  // namespace eros
#endif