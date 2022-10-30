#include <eros/SystemMonitor/Window/WindowTable.h>
namespace eros {
void WindowTable::setColumnLabels(std::vector<ColumnLabel> _columnLabels) {
    columnLabels = _columnLabels;
}
bool WindowTable::keyPressed(KeyMap key) {
    switch (key) {
        case KeyMap::KEY_down: return incrementSelectedRecord();
        case KeyMap::KEY_up: return decrementSelectedRecord();
        default: return true;
    }
}
bool WindowTable::incrementSelectedRecord() {
    int32_t newSelect = selectedRecordIndex + 1;
    if ((uint16_t)newSelect > (recordCount - 1)) {
        newSelect = recordCount - 1;
    }
    selectedRecordIndex = (uint16_t)newSelect;
    return true;
}
bool WindowTable::decrementSelectedRecord() {
    int32_t newSelect = selectedRecordIndex - 1;
    if (newSelect < 0) {
        newSelect = 0;
    }
    selectedRecordIndex = (uint16_t)newSelect;
    return true;
}
}  // namespace eros