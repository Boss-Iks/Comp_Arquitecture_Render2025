#include "../include/camera.hpp"
#include "../include/config.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

  inline void die(char const * msg) {
    std::cerr << msg << "\n";
    std::exit(EXIT_FAILURE);
  }

  inline std::array<double, 3> sub(std::array<double, 3> a, std::array<double, 3> b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  inline std::array<double, 3> add(std::array<double, 3> a, std::array<double, 3> b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  inline std::array<double, 3> mul(std::array<double, 3> a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
  }

  inline double dot(std::array<double, 3> a, std::array<double, 3> b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  inline std::array<double, 3> cross(std::array<double, 3> a, std::array<double, 3> b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  }

  inline double norm(std::array<double, 3> a) {
    return std::sqrt(dot(a, a));
  }

  inline std::array<double, 3> normalize(std::array<double, 3> a) {
    double const n = norm(a);
    if (n == 0.0) {
      die("Error: camera vectors produce zero-length basis.");
    }
    return mul(a, 1.0 / n);
  }

  // Compute image height from aspect ratio and image width, with validation.
  inline int compute_image_height(Config const & cfg) {
    if (cfg.aspect_w <= 0 or cfg.aspect_h <= 0) {
      die("Error: Invalid aspect ratio in config.");
    }
    double const h_double = static_cast<double>(cfg.image_width) *
                            static_cast<double>(cfg.aspect_h) /
                            static_cast<double>(cfg.aspect_w);
    int const h = static_cast<int>(std::round(h_double));
    if (h <= 0) {
      die("Error: Computed image height is non-positive.");
    }
    return h;
  }

  // Section 3.1 step 5: Build camera basis vectors following spec notation
  // vf = P - D (focal vector from target to camera)
  inline double build_camera_basis(Camera & cam) {
    auto const vf   = sub(cam.P, cam.D);  // Spec: vf = P - D (step 1)
    double const df = norm(vf);           // Spec: df = ||vf|| (step 2)

    auto const vf_hat = normalize(vf);  // vf normalized

    // Step 5: u = (n × vf) / ||n × vf||
    cam.u = normalize(cross(cam.N, vf_hat));

    // Step 5: v = vf × u
    cam.v = cross(vf_hat, cam.u);

    cam.vf_hat = vf_hat;

    return df;
  }

  // Section 3.1 steps 3-7: Compute projection window
  inline void compute_projection_window(Camera & cam, double df) {
    constexpr double pi = 3.14159265358979323846;  // NOLINT

    // Step 3: hp = 2·tan(θ/2)·df
    double const fov_rad = cam.fov_deg * (pi / 180.0);
    double const hp      = 2.0 * std::tan(fov_rad / 2.0) * df;

    // Step 4: wp = hp·(w/h)
    double const wp =
        hp * (static_cast<double>(cam.image_width) / static_cast<double>(cam.image_height));

    // Step 6: ph = wp·u, pv = -hp·v
    auto const ph = mul(cam.u, wp);
    auto const pv = mul(cam.v, -hp);  // Negative because image y goes down

    // Step 7: δx = ph/w, δy = pv/h
    auto const dx = mul(ph, 1.0 / static_cast<double>(cam.image_width));
    auto const dy = mul(pv, 1.0 / static_cast<double>(cam.image_height));

    // Step 7: O = P - vf - (1/2)ph - (1/2)pv + (1/2)δx + (1/2)δy
    // Since vf = P - D, we have -vf = D - P
    auto const vf = sub(cam.P, cam.D);
    auto const O =
        add(add(sub(sub(sub(cam.P, vf), mul(ph, 0.5)), mul(pv, 0.5)), mul(dx, 0.5)), mul(dy, 0.5));

    cam.O  = O;
    cam.dx = dx;
    cam.dy = dy;
  }

}  // namespace

Camera make_camera_from_config(Config const & cfg) {
  if (cfg.fov_deg <= 0.0 or cfg.fov_deg >= 180.0) {
    std::cerr << "Error: Invalid field_of_view in config.\n";
    std::exit(EXIT_FAILURE);
  }

  Camera cam{};
  cam.P            = cfg.cam_pos;
  cam.D            = cfg.cam_target;
  cam.N            = cfg.cam_north;
  cam.fov_deg      = cfg.fov_deg;
  cam.image_width  = cfg.image_width;
  cam.image_height = compute_image_height(cfg);

  // Background colours (gradient endpoints)
  cam.bg_dark  = cfg.bg_dark;
  cam.bg_light = cfg.bg_light;

  // Rendering parameters required by the project (section 2.2) now stored in Camera:
  cam.gamma             = cfg.gamma;
  cam.samples_per_pixel = cfg.samples_per_pixel;
  cam.max_depth         = cfg.max_depth;
  cam.material_rng_seed =
      static_cast<std::uint64_t>(static_cast<std::uint32_t>(cfg.material_rng_seed));
  cam.ray_rng_seed = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cfg.ray_rng_seed));

  double const df = build_camera_basis(cam);
  compute_projection_window(cam, df);

  return cam;
}
