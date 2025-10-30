#include "../include/camera.hpp"
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
    double n = norm(a);
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
    double h_double = static_cast<double>(cfg.image_width) *
                      static_cast<double>(cfg.aspect_h) /
                      static_cast<double>(cfg.aspect_w);
    int h = static_cast<int>(std::round(h_double));
    if (h <= 0) {
      die("Error: Computed image height is non-positive.");
    }
    return h;
  }

  inline double build_camera_basis(Camera & cam) {
    // Basis vectors
    auto vf    = sub(cam.D, cam.P);  // forward (to target)
    cam.vf_hat = normalize(vf);
    cam.u      = normalize(cross(cam.N, cam.vf_hat));  // right
    cam.v      = cross(cam.vf_hat, cam.u);             // up (orthonormal)

    // distance from camera origin to image plane center
    return norm(vf);
  }

  inline void compute_projection_window(Camera & cam, double df) {
    // Projection window geometry
    double fov_rad = cam.fov_deg * (M_PI / 180.0);
    double hp      = 2.0 * std::tan(fov_rad / 2.0) * df;  // physical height of plane
    double wp      = hp * (static_cast<double>(cam.image_width) /
                      static_cast<double>(cam.image_height));  // physical width of plane

    auto ph = mul(cam.u, wp);
    auto pv = mul(cam.v, -hp);  // minus: image y grows downward

    auto dx = mul(ph, 1.0 / static_cast<double>(cam.image_width));
    auto dy = mul(pv, 1.0 / static_cast<double>(cam.image_height));

    // Top-left pixel center
    auto vf = sub(cam.D, cam.P);
    auto Pc = add(cam.P, vf);
    auto O  = add(sub(sub(Pc, mul(ph, 0.5)), mul(pv, 0.5)), mul(add(dx, dy), 0.5));

    cam.O  = O;
    cam.dx = dx;
    cam.dy = dy;
  }

}  // anonymous namespace

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

  double df = build_camera_basis(cam);
  compute_projection_window(cam, df);

  return cam;
}
