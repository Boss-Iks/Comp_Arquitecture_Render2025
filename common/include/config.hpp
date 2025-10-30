#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <string_view>

struct Config {
  // Image / aspect
  int aspect_w    = 16;
  int aspect_h    = 9;
  int image_width = 1'920;
  double gamma    = 2.2;

  // Camera
  std::array<double, 3> cam_pos    = {0.0, 0.0, -10.0};
  std::array<double, 3> cam_target = {0.0, 0.0, 0.0};
  std::array<double, 3> cam_north  = {0.0, 1.0, 0.0};
  double fov_deg                   = 90.0;

  // Sampling
  int samples_per_pixel = 20;
  int max_depth         = 5;

  // RNG seeds (must be > 0 when we validate later)
  int material_rng_seed = 1;
  int ray_rng_seed      = 1;

  // Background colors [0,1]
  std::array<double, 3> bg_dark  = {0.25, 0.5, 1.0};
  std::array<double, 3> bg_light = {1.0, 1.0, 1.0};
};

// For now: only checks the file exists, returns defaults.
// We'll add full parsing in Step 2.B.
Config parse_config(std::string_view config_path);
