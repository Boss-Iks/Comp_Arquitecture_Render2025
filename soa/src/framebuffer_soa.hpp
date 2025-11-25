#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

// estructura SOA para framebuffer con canales RGB separados
struct FramebufferSOA {
  std::vector<std::uint8_t> R;
  std::vector<std::uint8_t> G;
  std::vector<std::uint8_t> B;
};

// inicializa framebuffer SOA con dimensiones dadas (relleno con ceros)
inline void initFramebufferSOA(FramebufferSOA & fb, int ancho, int alto) {
  std::size_t const n = static_cast<std::size_t>(ancho) * static_cast<std::size_t>(alto);
  fb.R.resize(n);
  fb.G.resize(n);
  fb.B.resize(n);
}

// calcula indice lineal desde coordenadas 2D para acceso SOA
[[nodiscard]] inline std::size_t idxSOA(std::size_t x, std::size_t y, std::size_t ancho) noexcept {
  return y * ancho + x;
}

// almacena valores RGB de pixel en SOA usando indice precalculado
inline void storePixelSOA(FramebufferSOA & fb, std::size_t idx, std::uint8_t r, std::uint8_t g,
                          std::uint8_t b) {
  fb.R[idx] = r;
  fb.G[idx] = g;
  fb.B[idx] = b;
}
