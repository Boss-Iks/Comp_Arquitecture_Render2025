#include "vector.hpp"

#include <cmath>

namespace render {

  double vector::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
  }

  //...

} // namespace render
