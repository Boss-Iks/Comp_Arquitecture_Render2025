#ifndef RENDER_VECTOR_HPP
#define RENDER_VECTOR_HPP

#include <cmath>

namespace render {

  class vector {
  public:
    vector(double cx, double cy, double cz) : x{cx}, y{cy}, z{cz} {}

    [[nodiscard]] double magnitude() const;

    //...
  private:
    double x, y, z;
  };

}  // namespace render

#endif
