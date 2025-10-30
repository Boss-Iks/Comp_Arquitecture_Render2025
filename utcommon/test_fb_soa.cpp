#include <gtest/gtest.h>
#include "framebuffer_soa.hpp"

TEST(FramebufferSOA, InitAndStore) {
    const int W = 4, H = 1;
    FramebufferSOA fb;
    initFramebufferSOA(fb, W, H);

    ASSERT_EQ(fb.R.size(), static_cast<size_t>(W*H));
    ASSERT_EQ(fb.G.size(), static_cast<size_t>(W*H));
    ASSERT_EQ(fb.B.size(), static_cast<size_t>(W*H));

    for (size_t k = 0; k < fb.R.size(); ++k) {
        EXPECT_EQ(fb.R[k], 0u);
        EXPECT_EQ(fb.G[k], 0u);
        EXPECT_EQ(fb.B[k], 0u);
    }

    storePixelSOA(fb, W, 2, 0, 7u, 8u, 9u);
    const size_t i = static_cast<size_t>(idxSOA(2, 0, W));
    EXPECT_EQ(fb.R[i], 7u);
    EXPECT_EQ(fb.G[i], 8u);
    EXPECT_EQ(fb.B[i], 9u);
}
