#ifndef RAYOS_HPP
#define RAYOS_HPP

#include "../../soa/src/framebuffer_soa.hpp"
#include <array>
#include <cstdint>
#include <vector>

struct Camera;
struct Scene;

struct Pixel {
  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;
};

struct Ray {
  std::array<double, 3> origin;
  std::array<double, 3> direction;
};

struct HitRecord {
  bool hit                     = false;
  double t                     = 1e10;
  std::array<double, 3> point  = {0.0, 0.0, 0.0};
  std::array<double, 3> normal = {0.0, 0.0, 0.0};
  std::uint32_t material_id    = 0;
};

void trace_rays_aos(Camera const & camara, Scene const & escena, std::vector<Pixel> & framebuffer);

void trace_rays_soa(Camera const & camara, Scene const & escena, FramebufferSOA & framebuffer);

#endif  // RAYOS_HPP
