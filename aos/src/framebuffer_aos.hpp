#pragma once
#include <cstdint>
#include <vector>

struct Pixel {
    std::uint8_t r, g, b;
};

inline void initFramebufferAOS(std::vector<Pixel>& fb, int width, int height) {
    fb.assign(static_cast<size_t>(width) * static_cast<size_t>(height), Pixel{0u,0u,0u});
}

inline int idxAOS(int x, int y, int width) { return y * width + x; }

inline void storePixelAOS(std::vector<Pixel>& fb, int width, int x, int y,
                          std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    fb[static_cast<size_t>(idxAOS(x,y,width))] = Pixel{r,g,b};
}
