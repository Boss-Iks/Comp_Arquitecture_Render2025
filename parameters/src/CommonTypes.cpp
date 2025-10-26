#include "CommonTypes.hpp"
#include <istream>

std::istream& operator>>(std::istream& is, Vec3& v) {
    // El stream fallará si no puede leer 3 doubles.
    is >> v.x >> v.y >> v.z;
    return is;
}

std::istream& operator>>(std::istream& is, Color& c) {
    // El stream fallará si no puede leer 3 doubles.
    is >> c.r >> c.g >> c.b;
    return is;
}

std::istream& operator>>(std::istream& is, AspectRatio& ar) {
    // El stream fallará si no puede leer 2 ints.
    is >> ar.width >> ar.height;
    return is;
}