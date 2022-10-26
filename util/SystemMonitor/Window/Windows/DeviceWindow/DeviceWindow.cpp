#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceWindow.h>
using namespace eros;
WindowSize DeviceWindow::getWindowSize() {
    WindowSize size;
    uint64_t minWidth = 0;
    for (auto column : getColumnLabels()) { minWidth += column.minWidth; }
    ScreenCoordinatePerc coord(CoordinateReference::GLOBAL, 50.0, 75.0, 50.0, 25.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = minWidth;
    return size;
}
std::vector<std::shared_ptr<IRecord>> DeviceWindow::getRecords() {
    std::vector<std::shared_ptr<IRecord>> records;
    uint16_t yValue = 0;
    {
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        uint16_t xValue = 0;
        uint16_t index = 0;
        {
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.color = Color::BLACK;
            data.data = "0";
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = yValue;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            // data.color = Color::UNKNOWN;
            data.data = "DummyDevice";
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = yValue;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            // data.color = Color::WHITE;

            data.data = "12.34";
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = yValue;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            // data.color = Color::RED;
            data.data = "56.78";
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = yValue;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            // data.color = Color::GREEN;
            data.data = "90.12";
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = yValue;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            // data.color = Color::PURPLE;
            data.data = "[1.234,5.678,9.012]";
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = yValue;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            // data.color = Color::YELLOW;
            data.data = "99.99";
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = yValue;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }

        record->setFields(fields);
        records.push_back(std::move(record));
        yValue++;
    }
    return records;
}
bool DeviceWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}