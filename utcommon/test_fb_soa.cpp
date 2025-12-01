#include "framebuffer_soa.hpp"
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>

TEST(FramebufferSOA, InitAndStore) {
  constexpr int ANCHO = 4;
  constexpr int ALTO  = 1;
  FramebufferSOA fb{};
  initFramebufferSOA(fb, ANCHO, ALTO);

  auto const tamanio_esperado = static_cast<std::size_t>(ANCHO) * static_cast<std::size_t>(ALTO);
  ASSERT_EQ(fb.R.size(), tamanio_esperado);
  ASSERT_EQ(fb.G.size(), tamanio_esperado);
  ASSERT_EQ(fb.B.size(), tamanio_esperado);

  for (std::size_t k = 0; k < fb.R.size(); ++k) {
    EXPECT_EQ(fb.R[k], std::uint8_t{0});
    EXPECT_EQ(fb.G[k], std::uint8_t{0});
    EXPECT_EQ(fb.B[k], std::uint8_t{0});
  }

  constexpr std::uint8_t ROJO  = 7;
  constexpr std::uint8_t VERDE = 8;
  constexpr std::uint8_t AZUL  = 9;
  constexpr std::size_t COL    = 2;
  constexpr std::size_t FILA   = 0;

  storePixelSOA(fb, idxSOA(COL, FILA, static_cast<std::size_t>(ANCHO)), {ROJO, VERDE, AZUL});

  std::size_t const idx = idxSOA(COL, FILA, static_cast<std::size_t>(ANCHO));
  EXPECT_EQ(fb.R[idx], ROJO);
  EXPECT_EQ(fb.G[idx], VERDE);
  EXPECT_EQ(fb.B[idx], AZUL);
}
