#ifndef SYSTEMMONITOR_DATASTRUCTURES_H
#define SYSTEMMONITOR_DATASTRUCTURES_H
#include <stdint.h>

#include <string>
namespace eros {
enum class KeyMap {
    KEY_None = 0,
    KEY_q = 113,
    KEY_Q = 81,
    KEY_TAB = 9,
    KEY_up = 65,
    KEY_down = 66,
};
enum class CoordinateReference { UNKNOWN = 0, GLOBAL = 1, TO_PARENT = 2, END_OF_LIST = 3 };
enum class Color {
    UNKNOWN = 0,
    RED = 1,
    GREEN = 2,
    BLUE = 3,
    WHITE = 4,
    BLACK = 5,
    YELLOW = 6,
    PURPLE = 7,
    END_OF_LIST = 8
};

struct ScreenCoordinatePerc {
    ScreenCoordinatePerc(
        CoordinateReference reference, double start_x, double start_y, double width, double height)
        : reference(reference),
          start_x_perc(start_x),
          start_y_perc(start_y),
          width_perc(width),
          height_perc(height) {
    }
    ScreenCoordinatePerc()
        : reference(CoordinateReference::UNKNOWN),
          start_x_perc(0.0),
          start_y_perc(0.0),
          width_perc(0.0),
          height_perc(0.0) {
    }
    CoordinateReference reference;
    double start_x_perc;
    double start_y_perc;
    double width_perc;
    double height_perc;
};
struct ScreenCoordinatePixel {
    ScreenCoordinatePixel(
        CoordinateReference reference, double start_x, double start_y, double width, double height)
        : reference(reference),
          start_x_pixel(start_x),
          start_y_pixel(start_y),
          width_pixel(width),
          height_pixel(height) {
    }
    ScreenCoordinatePixel()
        : reference(CoordinateReference::UNKNOWN),
          start_x_pixel(0),
          start_y_pixel(0),
          width_pixel(0),
          height_pixel(0) {
    }
    CoordinateReference reference;
    uint16_t start_x_pixel;
    uint16_t start_y_pixel;
    uint16_t width_pixel;
    uint16_t height_pixel;
};
class WindowSize
{
   public:
    WindowSize() : coordinate(), min_height_pixel(0), min_width_pixel(0) {
    }
    static ScreenCoordinatePixel convert(ScreenCoordinatePerc coord_perc,
                                         uint16_t width_pix,
                                         uint16_t height_pix) {
        ScreenCoordinatePixel coord(coord_perc.reference, 0, 0, 0, 0);
        coord.start_x_pixel = (uint16_t)((double)width_pix * (.01 * coord_perc.start_x_perc));
        coord.start_y_pixel = (uint16_t)((double)height_pix * (.01 * coord_perc.start_y_perc));
        coord.width_pixel = (uint16_t)((double)width_pix * (.01 * coord_perc.width_perc));
        coord.height_pixel = (uint16_t)((double)height_pix * (.01 * coord_perc.height_perc));
        return coord;
    }

    ScreenCoordinatePerc coordinate;
    uint16_t min_height_pixel;
    uint16_t min_width_pixel;
};
struct RenderData {
    RenderData(ScreenCoordinatePixel startCoordinate, Color color, std::string data)
        : startCoordinate(startCoordinate), color(color), data(data) {
    }
    RenderData() : startCoordinate(), color(Color::UNKNOWN), data("") {
    }
    ScreenCoordinatePixel startCoordinate;
    Color color;
    std::string data;
};
}  // namespace eros
#endif