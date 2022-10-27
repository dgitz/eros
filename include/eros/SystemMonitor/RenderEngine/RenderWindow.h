#ifndef RENDERWINDOW_H
#define RENDERWINDOW_H
#include <curses.h>
#include <eros/Logger.h>
#include <eros/SystemMonitor/DataStructures.h>
#include <eros/SystemMonitor/Window/WindowTable.h>
#include <eros/SystemMonitor/Window/WindowText.h>
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
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            delete window_reference;
            window_reference = nullptr;
            // LCOV_EXCL_STOP
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
    bool isFocused() {
        return focused;
    }
    void setFocused(bool v) {
        focused = v;
    }

   private:
    eros::Logger* logger;
    WINDOW* window_reference;
    ScreenCoordinatePixel actualSize;
    ScreenCoordinatePixel minSize;
    bool focused;
};
}  // namespace eros
#endif