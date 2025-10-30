// utcommon/test_camera.cpp
#include "../common/include/camera.hpp"
#include "../common/include/config.hpp"
#include <array>
#include <cmath>  // for std::sqrt
#include <gtest/gtest.h>

// --- Basic smoke test to confirm framework works ---
TEST(SmokeTest, Runs) {
  SUCCEED();
}

// --- Test that make_camera_from_config builds correctly ---
TEST(CameraTest, BuildsFromValidConfig) {
  // Arrange: create a valid config
  Config cfg;
  cfg.aspect_w    = 16;
  cfg.aspect_h    = 9;
  cfg.image_width = 800;
  cfg.fov_deg     = 90.0;
  cfg.cam_pos     = {0.0, 0.0, -5.0};
  cfg.cam_target  = {0.0, 0.0, 0.0};
  cfg.cam_north   = {0.0, 1.0, 0.0};

  // Act
  Camera cam = make_camera_from_config(cfg);

  // Assert basic properties
  EXPECT_EQ(cam.image_width, 800);
  EXPECT_EQ(cam.image_height, 450);  // 800 * 9 / 16

  // Check that direction vectors are normalized
  auto len = [](std::array<double, 3> const & v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  };
  EXPECT_NEAR(len(cam.vf_hat), 1.0, 1e-9);
  EXPECT_NEAR(len(cam.u), 1.0, 1e-9);
  EXPECT_NEAR(len(cam.v), 1.0, 1e-9);

  // Optional: sanity check that projection plane origin isn’t degenerate
  EXPECT_FALSE(std::isnan(cam.O[0]));
  EXPECT_FALSE(std::isnan(cam.O[1]));
  EXPECT_FALSE(std::isnan(cam.O[2]));
}
