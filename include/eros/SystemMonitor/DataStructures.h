#ifndef SYSTEMMONITOR_DATASTRUCTURES_H
#define SYSTEMMONITOR_DATASTRUCTURES_H
#include <stdint.h>

#include <string>
namespace eros {
enum class KeyMap { KEY_q = 113, KEY_Q = 81, KEY_TAB = 9 };
struct ScreenCoordinatePerc {
    ScreenCoordinatePerc(double start_x, double start_y, double width, double height)
        : start_x_perc(start_x), start_y_perc(start_y), width_perc(width), height_perc(height) {
    }
    ScreenCoordinatePerc()
        : start_x_perc(0.0), start_y_perc(0.0), width_perc(0.0), height_perc(0.0) {
    }
    double start_x_perc;
    double start_y_perc;
    double width_perc;
    double height_perc;
};
struct ScreenCoordinatePixel {
    ScreenCoordinatePixel(double start_x, double start_y, double width, double height)
        : start_x_pixel(start_x), start_y_pixel(start_y), width_pixel(width), height_pixel(height) {
    }
    ScreenCoordinatePixel() : start_x_pixel(0), start_y_pixel(0), width_pixel(0), height_pixel(0) {
    }
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
        ScreenCoordinatePixel coord(0, 0, 0, 0);
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
    uint16_t x;
    uint16_t y;
    std::string data;
};
}  // namespace eros
#endif