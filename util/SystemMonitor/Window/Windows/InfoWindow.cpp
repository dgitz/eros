#include <eros/SystemMonitor/Window/Windows/InfoWindow.h>
using namespace eros;
WindowSize InfoWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(33.0, 75.0, 33.0, 25.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = 30;
    return size;
}
std::vector<RenderData> InfoWindow::getData() {
    std::vector<RenderData> dataVector;
    RenderData data;
    data.data = "Info";
    data.x = 0;
    data.y = 0;
    dataVector.push_back(data);
    return dataVector;
}
bool InfoWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}