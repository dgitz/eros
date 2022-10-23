#include <eros/SystemMonitor/Window/Windows/DeviceWindow.h>
using namespace eros;
WindowSize DeviceWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(66.0, 75.0, 34.0, 25.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = 30;
    return size;
}
std::vector<IRecord*> DeviceWindow::getRecords() {
    std::vector<IRecord*> records;
    {
        std::vector<IField*> fields;
        GenericRecord* record = new GenericRecord();
        GenericField* field = new GenericField();
        RenderData data;
        data.data = "DeviceList";
        data.x = 0;
        data.y = 0;
        field->setData(data);
        fields.push_back(field);

        record->setFields(fields);
        records.push_back(record);
    }

    return records;
}
bool DeviceWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}