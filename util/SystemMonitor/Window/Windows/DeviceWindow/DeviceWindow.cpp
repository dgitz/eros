#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceWindow.h>
using namespace eros;
WindowSize DeviceWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(66.0, 75.0, 34.0, 25.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = 30;
    return size;
}
std::vector<std::shared_ptr<IRecord>> DeviceWindow::getRecords() {
    std::vector<std::shared_ptr<IRecord>> records;
    {
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "DeviceList";
        data.x = 0;
        data.y = 0;
        field->setData(data);
        fields.push_back(std::move(field));

        record->setFields(fields);
        records.push_back(std::move(record));
    }
    return records;
}
bool DeviceWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}