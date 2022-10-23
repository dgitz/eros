#include <eros/SystemMonitor/Window/Windows/HeaderWindow.h>
using namespace eros;
WindowSize HeaderWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(0.0, 0.0, 100.0, 15.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = 30;
    return size;
}
std::vector<RenderData> HeaderWindow::getData() {
    std::vector<RenderData> dataVector;
    RenderData data;
    data.data = "Header";
    data.x = 0;
    data.y = 0;
    dataVector.push_back(data);
    return dataVector;
}