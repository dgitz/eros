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
std::vector<IRecord*> NodeDiagnosticsWindow::getRecords() {
    std::vector<IRecord*> records;
    {
        std::vector<IField*> fields;
        GenericRecord* record = new GenericRecord();
        GenericField* field = new GenericField();
        RenderData data;
        data.data = "NodeDiagnostics";
        data.x = 0;
        data.y = 0;
        field->setData(data);
        fields.push_back(field);

        record->setFields(fields);
        records.push_back(record);
    }

    return records;
}
bool NodeDiagnosticsWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}