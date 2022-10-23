#include <eros/SystemMonitor/Window/Windows/NodeDiagnosticsWindow.h>
using namespace eros;
WindowSize NodeDiagnosticsWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(70.0, 15.0, 30.0, 60.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = 30;
    return size;
}
std::vector<RenderData> NodeDiagnosticsWindow::getData() {
    std::vector<RenderData> dataVector;
    RenderData data;
    data.data = "NodeDiagnostics";
    data.x = 0;
    data.y = 0;
    dataVector.push_back(data);
    return dataVector;
}