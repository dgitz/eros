#ifndef SYSTEMMONITOR_DATASTRUCTURES_H
#define SYSTEMMONITOR_DATASTRUCTURES_H
namespace eros {
enum class CoordinateReference { UNKNOWN = 0, GLOBAL = 1, TOPARENT = 2 };
enum class CoordinateType { UNKNOWN = 0, PIXEL = 1, PERCENTAGE = 2 };
struct WindowSize {
    WindowSize() : coordinateType(CoordinateType::UNKNOWN), x(0.0), y(0.0) {
    }

    CoordinateType coordinateType;
    double x;
    double y;
};
}  // namespace eros
#endif