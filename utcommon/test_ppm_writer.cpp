#include <gtest/gtest.h>
#include "ppm_writer.hpp"
#include <vector>
#include <cstdio>

struct Pixel { unsigned char r,g,b; };
struct FramebufferSOA { std::vector<unsigned char> R,G,B; };

TEST(PPMWriter, AOS_WritesHeaderAndBody) {
    std::vector<Pixel> fb(4, Pixel{10,20,30});
    const char* path = "test_aos.ppm";
    ASSERT_TRUE(writePPM_AOS(path, fb, 2, 2));
    std::FILE* f = std::fopen(path, "r");
    ASSERT_TRUE(f != nullptr);
    char header[64];
    ASSERT_TRUE(std::fgets(header, sizeof(header), f));
    EXPECT_EQ(std::string(header), "P3\n");
    std::fclose(f);
    std::remove(path);
}

TEST(PPMWriter, SOA_WritesHeaderAndBody) {
    FramebufferSOA fb;
    fb.R.assign(4, 10); fb.G.assign(4, 20); fb.B.assign(4, 30);
    const char* path = "test_soa.ppm";
    ASSERT_TRUE(writePPM_SOA(path, fb, 2, 2));
    std::FILE* f = std::fopen(path, "r");
    ASSERT_TRUE(f != nullptr);
    char header[64];
    ASSERT_TRUE(std::fgets(header, sizeof(header), f));
    EXPECT_EQ(std::string(header), "P3\n");
    std::fclose(f);
    std::remove(path);
}
