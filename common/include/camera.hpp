#pragma once
#include <array>
#include <string_view>

#include "config.hpp"  // uses your parsed config

struct Camera {
  // Inputs (copied from config)
  std::array<double, 3> P{};  // camera position
  std::array<double, 3> D{};  // camera target
  std::array<double, 3> N{};  // camera "north"/up
  double fov_deg{};
  int image_width{};
  int image_height{};

  // Derived basis & pixel steps
  std::array<double, 3> vf_hat{};  // forward (unit)
  std::array<double, 3> u{};       // right (unit)
  std::array<double, 3> v{};       // up (unit)

  std::array<double, 3> O{};   // top-left pixel center of projection plane
  std::array<double, 3> dx{};  // step to next pixel in x
  std::array<double, 3> dy{};  // step to next pixel in y
};

// Throws process-terminating errors (stderr + exit) on invalid inputs.
Camera make_camera_from_config(Config const & cfg);
