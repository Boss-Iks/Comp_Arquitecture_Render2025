#include <gtest/gtest.h>
#include <vector>
#include "framebuffer_aos.hpp"

TEST(FramebufferAOS, InitAndStore) {
    const int W = 3, H = 2;
    std::vector<Pixel> fb;
    initFramebufferAOS(fb, W, H);
    ASSERT_EQ(fb.size(), static_cast<size_t>(W*H));

    for (const auto& p : fb) {
        EXPECT_EQ(p.r, 0u);
        EXPECT_EQ(p.g, 0u);
        EXPECT_EQ(p.b, 0u);
    }

    storePixelAOS(fb, W, 1, 0, 10u, 20u, 30u);
    const int i = idxAOS(1, 0, W);
    EXPECT_EQ(fb[static_cast<size_t>(i)].r, 10u);
    EXPECT_EQ(fb[static_cast<size_t>(i)].g, 20u);
    EXPECT_EQ(fb[static_cast<size_t>(i)].b, 30u);
}
