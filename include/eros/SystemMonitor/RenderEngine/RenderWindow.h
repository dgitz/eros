#ifndef RENDERWINDOW_H
#define RENDERWINDOW_H
#include <curses.h>
#include <eros/Logger.h>
#include <eros/SystemMonitor/DataStructures.h>
namespace eros {
WINDOW* create_newwin(int height, int width, int starty, int startx);

class RenderWindow
{
   public:
    RenderWindow(eros::Logger* logger,
                 WindowSize size,
                 uint16_t mainWindowWidth,
                 uint16_t mainWindowHeight);
    virtual ~RenderWindow() {
        if (window_reference != nullptr) {
            delete window_reference;
            window_reference = nullptr;
        }
    }
    bool init();
    void set_window_reference(WINDOW* win) {
        window_reference = win;
    }
    WINDOW* get_window_reference() {
        return window_reference;
    }
    ScreenCoordinatePixel getActualSize() {
        return actualSize;
    }

   private:
    eros::Logger* logger;
    WINDOW* window_reference;
    ScreenCoordinatePixel actualSize;
    ScreenCoordinatePixel minSize;
};
}  // namespace eros
#endif