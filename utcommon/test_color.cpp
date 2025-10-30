#include <gtest/gtest.h>
#include "color.hpp"

TEST(Color, FloatToByteBasic){
  EXPECT_EQ(floatToByte(0.0f), 0u);
  EXPECT_EQ(floatToByte(0.9999f), 255u);
}

TEST(Color, AverageAndGamma){
  std::vector<ColorRGB> s{{1.f,0.f,0.f},{0.f,1.f,0.f}};
  ColorRGB avg = averageSamples(s);
  ByteRGB px = finalizePixel(avg, 2.2f);
  EXPECT_GT(px.r,100u); EXPECT_GT(px.g,100u); EXPECT_LT(px.b,5u);
}
