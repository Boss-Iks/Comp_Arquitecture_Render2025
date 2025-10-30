#pragma once
#include <cstdint>
#include <vector>

struct FramebufferSOA {
    std::vector<std::uint8_t> R, G, B;
};

inline void initFramebufferSOA(FramebufferSOA& fb, int width, int height) {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    fb.R.assign(n, 0u);
    fb.G.assign(n, 0u);
    fb.B.assign(n, 0u);
}

inline int idxSOA(int x, int y, int width) { return y * width + x; }

inline void storePixelSOA(FramebufferSOA& fb, int width, int x, int y,
                          std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    const size_t i = static_cast<size_t>(idxSOA(x,y,width));
    fb.R[i] = r; fb.G[i] = g; fb.B[i] = b;
}
