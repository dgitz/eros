#include <eros/SystemMonitor/Window/WindowTable.h>
namespace eros {
void WindowTable::setColumnLabels(std::vector<ColumnLabel> _columnLabels) {
    columnLabels = _columnLabels;
}
}  // namespace eros