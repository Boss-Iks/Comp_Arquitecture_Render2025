#pragma once
#include <string>
#include <vector>

struct Pixel;           // AOS
struct FramebufferSOA;  // SOA

// escribe framebuffer AOS a archivo PPM en formato P3
bool writePPM_AOS(std::string const & ruta, std::vector<Pixel> const & fb, int ancho, int alto);

// escribe framebuffer SOA a archivo PPM en formato P3
bool writePPM_SOA(std::string const & ruta, FramebufferSOA const & fb, int ancho, int alto);
