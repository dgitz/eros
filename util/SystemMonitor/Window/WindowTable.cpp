#include <eros/SystemMonitor/Window/WindowTable.h>
namespace eros {
std::string WindowTable::getData() {
    return "Dumb";
}
WindowSize WindowTable::getWindowSize() {
    WindowSize size;
    size.coordinateType = CoordinateType::PERCENTAGE;
    size.x = 100.0;
    size.y = 100.0;
    return size;
}
}  // namespace eros