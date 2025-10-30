// utcommon/test_camera.cpp
#include "../common/include/camera.hpp"
#include "../common/include/config.hpp"
#include <array>
#include <cmath>    // for std::sqrt
#include <cstdlib>  // for EXIT_FAILURE
#include <gtest/gtest.h>

// ---------- VALID CAMERA CREATION ----------
TEST(CameraTest, BuildsFromValidConfig) {
  Config cfg;
  cfg.aspect_w    = 16;
  cfg.aspect_h    = 9;
  cfg.image_width = 800;
  cfg.fov_deg     = 60.0;
  cfg.cam_pos     = {0.0, 0.0, -5.0};
  cfg.cam_target  = {0.0, 0.0, 0.0};
  cfg.cam_north   = {0.0, 1.0, 0.0};

  Camera cam = make_camera_from_config(cfg);

  EXPECT_EQ(cam.image_width, 800);
  EXPECT_EQ(cam.image_height, 450);

  auto len = [](std::array<double, 3> const & v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  };

  EXPECT_NEAR(len(cam.vf_hat), 1.0, 1e-9);
  EXPECT_NEAR(len(cam.u), 1.0, 1e-9);
  EXPECT_NEAR(len(cam.v), 1.0, 1e-9);
  EXPECT_FALSE(std::isnan(cam.O[0]));
}

// ---------- INVALID FOV ----------
TEST(CameraTest, InvalidFovDies) {
  Config cfg;
  cfg.aspect_w    = 1;
  cfg.aspect_h    = 1;
  cfg.image_width = 100;
  cfg.fov_deg     = -10.0;
  cfg.cam_pos     = {0.0, 0.0, 0.0};
  cfg.cam_target  = {0.0, 0.0, 1.0};
  cfg.cam_north   = {0.0, 1.0, 0.0};

  EXPECT_EXIT(make_camera_from_config(cfg), ::testing::ExitedWithCode(EXIT_FAILURE),
              "Invalid field_of_view");
}
