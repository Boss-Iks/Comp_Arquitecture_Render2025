#include <print>

#include "vector.hpp"

int main() {
  std::println("Starting SOA rendering");
  render::vector vec{1.0, 2.0, 3.0};
  std::println("Vector magnitude: {}", vec.magnitude());
}
