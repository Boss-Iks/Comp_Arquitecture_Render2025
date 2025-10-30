// utcommon/test_config.cpp
#include "../common/include/config.hpp"
#include <cstdlib>  // EXIT_FAILURE
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

  // helper must be in anonymous namespace (clang-tidy asked for this)
  std::string make_temp_config(std::string const & content) {
    // we don't modify this name later → const
    std::string const path = "tmp_test_config.txt";
    std::ofstream out(path);
    out << content;
    return path;
  }

}  // namespace

TEST(ConfigTest, ParsesValidConfig) {
  std::string const path = make_temp_config(R"(
    aspect_ratio: 16 9
    image_width: 800
    field_of_view: 60
    camera_position: 0 0 -5
    camera_target: 0 0 0
    camera_north: 0 1 0
  )");

  Config const cfg = parse_config(path);

  EXPECT_EQ(cfg.aspect_w, 16);
  EXPECT_EQ(cfg.aspect_h, 9);
  EXPECT_EQ(cfg.image_width, 800);
  EXPECT_DOUBLE_EQ(cfg.fov_deg, 60.0);
}

TEST(ConfigTest, UnknownKeyDies) {
  std::string const path = make_temp_config(R"(foo_bar: 123)");
  EXPECT_EXIT(parse_config(path), ::testing::ExitedWithCode(EXIT_FAILURE),
              "Unknown configuration key");
}

TEST(ConfigTest, InvalidFieldOfViewDies) {
  std::string const path = make_temp_config(R"(
    field_of_view: -1
  )");
  EXPECT_EXIT(parse_config(path), ::testing::ExitedWithCode(EXIT_FAILURE), "Invalid value");
}
