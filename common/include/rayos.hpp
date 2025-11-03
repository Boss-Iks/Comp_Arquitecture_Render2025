#ifndef RAYOS_HPP
#define RAYOS_HPP

#include <array>
#include <cstdint>
#include <vector>

// Forward declarations for common types
struct CameraConfig;
struct SceneData;

// for aos
struct Pixel {
  std::uint8_t r, g, b;
};

struct Ray {
  std::array<double, 3> origin;
  std::array<double, 3> direction;
};

struct HitRecord {
  bool hit = false;
  double t = 1e10;
  std::array<double, 3> point;
  std::array<double, 3> normal;
  uint32_t material_id = 0;
};

void trace_rays_aos(std::array<float, 3> const & origen_ventana,
                    std::array<float, 3> const & paso_x, std::array<float, 3> const & paso_y,
                    std::array<double, 3> const & camera_position, SceneData const & scene,
                    int image_width, int image_height, std::vector<Pixel> & framebuffer);

// for soa

struct FramebufferSOA {
  std::vector<std::uint8_t> R;
  std::vector<std::uint8_t> G;
  std::vector<std::uint8_t> B;
};

void trace_rays_soa(std::array<float, 3> const & origen_ventana,
                    std::array<float, 3> const & paso_x, std::array<float, 3> const & paso_y,
                    std::array<double, 3> const & camera_position, SceneData const & scene,
                    int image_width, int image_height, FramebufferSOA & framebuffer);

#endif  // RAYOS_HPP
