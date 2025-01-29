#pragma once
namespace eros_nodes::SystemMonitor {
const double COMMTIMEOUT_THRESHOLD = 5.0f;
/*! \struct ScreenCoordinatePerc
    \brief ScreenCoordinatePerc container.
    */
struct ScreenCoordinatePerc {
    ScreenCoordinatePerc(double start_x, double start_y, double width, double height)
        : start_x_perc(start_x), start_y_perc(start_y), width_perc(width), height_perc(height) {
    }
    double start_x_perc;
    double start_y_perc;
    double width_perc;
    double height_perc;
};  // namespace eros_nodes::SystemMonitorstructScreenCoordinatePerc
/*! \struct ScreenCoordinatePixel
\brief ScreenCoordinatePixel container.
*/
struct ScreenCoordinatePixel {
    ScreenCoordinatePixel(double start_x, double start_y, double width, double height)
        : start_x_pix(start_x), start_y_pix(start_y), width_pix(width), height_pix(height) {
    }
    uint16_t start_x_pix;
    uint16_t start_y_pix;
    uint16_t width_pix;
    uint16_t height_pix;
};
enum class Color {
    UNKNOWN = 0,
    NO_COLOR = 1,
    RED_COLOR = 2,
    YELLOW_COLOR = 3,
    GREEN_COLOR = 4,
    BLUE_COLOR = 5,
    GRAY_COLOR = 6,
    PURPLE_COLOR = 7,
    END_OF_LIST = 8
};
/*! \struct Field
    \brief Field container, used for holding Field attributes.
    */
struct Field {
    Field(std::string _text, uint16_t _width) : text(_text), width(_width) {
    }
    std::string text;
    std::size_t width;
};
}  // namespace eros_nodes::SystemMonitor