#pragma once
#include <curses.h>

#include "WindowDefinitions.h"
namespace eros_nodes::SystemMonitor {
class SystemMonitorUtility
{
   public:
    static WINDOW* create_newwin(int height, int width, int starty, int startx);
    static ScreenCoordinatePixel convertCoordinate(ScreenCoordinatePerc coord_perc,
                                                   uint16_t width_pix,
                                                   uint16_t height_pix);
};

}  // namespace eros_nodes::SystemMonitor