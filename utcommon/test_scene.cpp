// utcommon/test_scene.cpp
#include "../common/include/scene.hpp"
#include <cstdlib>  // EXIT_FAILURE
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace {

  std::string make_temp_scene(std::string const & content) {
    std::string const path = "tmp_test_scene.txt";
    std::ofstream out(path);
    out << content;
    return path;
  }

}  // namespace

TEST(SceneTest, ParsesMatteMaterial) {
  std::string const path = make_temp_scene(R"(
    matte: red 0.5 0.5 0.5
  )");

  Scene const scn = parse_scene(path);

  ASSERT_EQ(scn.materials.size(), 1);
  EXPECT_EQ(scn.materials[0].name, "red");
  EXPECT_DOUBLE_EQ(scn.materials[0].matte.rgb[0], 0.5);
}

TEST(SceneTest, DuplicateMaterialDies) {
  std::string const path = make_temp_scene(R"(
    matte: red 0.1 0.1 0.1
    matte: red 0.2 0.2 0.2
  )");

  EXPECT_EXIT(parse_scene(path), ::testing::ExitedWithCode(EXIT_FAILURE), "already exists");
}

TEST(SceneTest, InvalidSphereDies) {
  std::string const path = make_temp_scene(R"(
    matte: red 0.1 0.1 0.1
    sphere: 0 0 0 -5 red
  )");

  EXPECT_EXIT(parse_scene(path), ::testing::ExitedWithCode(EXIT_FAILURE), "Invalid object");
}
