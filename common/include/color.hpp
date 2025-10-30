#ifndef COLOR_HPP
#define COLOR_HPP

#include <cstdint>
#include <cmath>
#include <vector>



// High-precision linear RGB in [0..1] (pre-gamma, float space)
struct ColorRGB {
    float r;
    float g;
    float b;
};

// Final 8-bit pixel (post-processing)
struct ByteRGB {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
};



inline float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

// Convert float [0..1] → byte [0..255] using 0.999 cap to avoid 256
inline std::uint8_t floatToByte(float c) {
    // keep inside [0, 0.999] before scaling
    if (c < 0.0f) c = 0.0f;
    if (c > 0.999f) c = 0.999f;
    // 256 * 0.999 = 255.744 → trunc to 255
    return static_cast<std::uint8_t>(c * 256.0f);
}



// Average N samples for one pixel.
// If samples is empty, return black.
inline ColorRGB averageSamples(const std::vector<ColorRGB>& samples) {
    if (samples.empty()) {
        return {0.0f, 0.0f, 0.0f};
    }
    float accR = 0.0f;
    float accG = 0.0f;
    float accB = 0.0f;
    for (const ColorRGB& s : samples) {
        accR += s.r;
        accG += s.g;
        accB += s.b;
    }
    const float invN = 1.0f / static_cast<float>(samples.size());
    return { accR * invN, accG * invN, accB * invN };
}

// In-place running accumulation (avoids vectors/alloc):
//   acc += sample; later divide by count.
inline void accumulateSample(ColorRGB& acc, const ColorRGB& sample) {
    acc.r += sample.r;
    acc.g += sample.g;
    acc.b += sample.b;
}

// Compute average from an accumulated sum and sample count.
inline ColorRGB averageFromSum(const ColorRGB& sum, int count) {
    if (count <= 0) return {0.0f, 0.0f, 0.0f};
    const float invN = 1.0f / static_cast<float>(count);
    return { sum.r * invN, sum.g * invN, sum.b * invN };
}



// Apply gamma ( >0 ). If gamma <= 0, skip (treat as linear).
inline ColorRGB applyGamma(const ColorRGB& linear, float gamma) {
    if (gamma <= 0.0f) {
        // no correction
        return { clamp01(linear.r), clamp01(linear.g), clamp01(linear.b) };
    }
    const float invGamma = 1.0f / gamma;
    auto g = [invGamma](float c) -> float {
        c = clamp01(c);
        // std::pow handles 0..1 safely
        return std::pow(c, invGamma);
    };
    return { g(linear.r), g(linear.g), g(linear.b) };
}

// Full finalize pipeline for one pixel:
// (avg linear) -> gamma -> clamp -> 8-bit pack
inline ByteRGB finalizePixel(const ColorRGB& averagedLinear, float gamma) {
    const ColorRGB corrected = applyGamma(averagedLinear, gamma);
    return ByteRGB{
        floatToByte(corrected.r),
        floatToByte(corrected.g),
        floatToByte(corrected.b)
    };
}

#endif // COLOR_HPP
