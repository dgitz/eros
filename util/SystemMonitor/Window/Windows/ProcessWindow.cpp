#include <eros/SystemMonitor/Window/Windows/ProcessWindow.h>
using namespace eros;
WindowSize ProcessWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(0.0, 15.0, 70.0, 60.0);
    size.coordinate = coord;
    size.min_height_pixel = 8;
    size.min_width_pixel = 60;
    return size;
}
std::vector<IRecord*> ProcessWindow::getRecords() {
    std::vector<IRecord*> records;
    {
        std::vector<IField*> fields;
        GenericRecord* record = new GenericRecord();
        GenericField* field = new GenericField();
        RenderData data;
        data.data = "Process: " + std::to_string(updateCounter);
        data.x = 0;
        data.y = 0;
        field->setData(data);
        fields.push_back(field);

        record->setFields(fields);
        records.push_back(record);
    }

    return records;
}
bool ProcessWindow::new_heartbeat(eros::heartbeat msg) {
    (void)msg;
    updateCounter++;
    return true;
}
bool ProcessWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}