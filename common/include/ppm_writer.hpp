#pragma once
#include <string>
#include <vector>
#include <cstdint>

struct Pixel;          // AOS
struct FramebufferSOA; // SOA

bool writePPM_AOS(const std::string& path,
                  const std::vector<Pixel>& fb,
                  int width, int height);

bool writePPM_SOA(const std::string& path,
                  const FramebufferSOA& fb,
                  int width, int height);
