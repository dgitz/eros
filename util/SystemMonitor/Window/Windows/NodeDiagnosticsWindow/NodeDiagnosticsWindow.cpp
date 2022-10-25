#include <eros/SystemMonitor/Window/Windows/NodeDiagnosticsWindow/NodeDiagnosticsWindow.h>
using namespace eros;
WindowSize NodeDiagnosticsWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(CoordinateReference::GLOBAL, 70.0, 15.0, 30.0, 60.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = 30;
    return size;
}
std::vector<std::shared_ptr<IRecord>> NodeDiagnosticsWindow::getRecords() {
    std::vector<std::shared_ptr<IRecord>> records;
    {
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "NodeDiagnostics";
        data.startCoordinate.start_x_pixel = 0;
        data.startCoordinate.start_y_pixel = 0;
        field->setData(data);
        fields.push_back(std::move(field));

        record->setFields(fields);
        records.push_back(std::move(record));
    }

    return records;
}
bool NodeDiagnosticsWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}