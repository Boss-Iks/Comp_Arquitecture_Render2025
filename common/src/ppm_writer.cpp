#include "ppm_writer.hpp"
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <cstdint>

// local shadow structs so we don't pull headers across targets
struct Pixel { std::uint8_t r, g, b; };
struct FramebufferSOA { std::vector<std::uint8_t> R, G, B; };

static void write_header(std::FILE* f, int w, int h) {
    std::fprintf(f, "P3\n%d %d\n255\n", w, h);
}

static void write_triplet(std::FILE* f, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    std::fprintf(f, "%u %u %u\n",
                 static_cast<unsigned>(r),
                 static_cast<unsigned>(g),
                 static_cast<unsigned>(b));
}

bool writePPM_AOS(const std::string& path,
                  const std::vector<Pixel>& fb,
                  int width, int height)
{
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error(std::string("fopen failed: ") + std::strerror(errno));

    write_header(f, width, height);
    for (int y = 0; y < height; ++y) {
        const int row = y * width;
        for (int x = 0; x < width; ++x) {
            const Pixel& p = fb[static_cast<size_t>(row + x)];
            write_triplet(f, p.r, p.g, p.b);
        }
    }
    std::fclose(f);
    return true;
}

bool writePPM_SOA(const std::string& path,
                  const FramebufferSOA& fb,
                  int width, int height)
{
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error(std::string("fopen failed: ") + std::strerror(errno));

    write_header(f, width, height);
    for (int y = 0; y < height; ++y) {
        const int row = y * width;
        for (int x = 0; x < width; ++x) {
            const size_t i = static_cast<size_t>(row + x);
            write_triplet(f, fb.R[i], fb.G[i], fb.B[i]);
        }
    }
    std::fclose(f);
    return true;
}
