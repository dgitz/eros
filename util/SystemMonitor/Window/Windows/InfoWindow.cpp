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
std::vector<IRecord*> InfoWindow::getRecords() {
    std::vector<IRecord*> records;
    {
        std::vector<IField*> fields;
        GenericRecord* record = new GenericRecord();
        GenericField* field = new GenericField();
        RenderData data;
        data.data = "Info";
        data.x = 0;
        data.y = 0;
        field->setData(data);
        fields.push_back(field);

        record->setFields(fields);
        records.push_back(record);
    }

    return records;
}
bool InfoWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}