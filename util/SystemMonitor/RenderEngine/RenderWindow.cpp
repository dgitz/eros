#include <eros/SystemMonitor/RenderEngine/RenderWindow.h>
namespace eros {
WINDOW* create_newwin(int height, int width, int starty, int startx) {
    WINDOW* local_win;

    local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0); /* 0, 0 gives default characters
                           * for the vertical and horizontal
                           * lines			*/
    wrefresh(local_win);  /* Show that box 		*/

    return local_win;
}
RenderWindow::RenderWindow(eros::Logger* logger,
                           WindowSize size,
                           uint16_t mainWindowWidth,
                           uint16_t mainWindowHeight)
    : logger(logger), focused(false) {
    ScreenCoordinatePixel coord =
        WindowSize::convert(size.coordinate, mainWindowWidth, mainWindowHeight);
    WINDOW* win = create_newwin(
        coord.height_pixel, coord.width_pixel, coord.start_y_pixel, coord.start_x_pixel);
    actualSize = coord;
    minSize.height_pixel = size.min_height_pixel;
    minSize.width_pixel = size.min_width_pixel;
    set_window_reference(win);
}
bool RenderWindow::init() {
    if (actualSize.height_pixel < minSize.height_pixel) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        logger->enable_consoleprint();
        logger->log_error("Height Too Small: " + std::to_string(actualSize.height_pixel) + "<" +
                          std::to_string(minSize.height_pixel));

        return false;
        // LCOV_EXCL_STOP
    }
    if (actualSize.width_pixel < minSize.width_pixel) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        logger->enable_consoleprint();
        logger->log_error("Width Too Small: " + std::to_string(actualSize.width_pixel) + "<" +
                          std::to_string(minSize.width_pixel));

        return false;
        // LCOV_EXCL_STOP
    }
    return true;
}
}  // namespace eros