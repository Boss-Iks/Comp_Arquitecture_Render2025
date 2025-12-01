#include "ppm_writer.hpp"
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <print>
#include <stdexcept>
#include <string>
#include <vector>

// estructuras locales para evitar dependencias de headers externos
struct Pixel {
  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;
};

struct FramebufferSOA {
  std::vector<std::uint8_t> R;
  std::vector<std::uint8_t> G;
  std::vector<std::uint8_t> B;
};

namespace {

  // deleter personalizado para FILE* usado con unique_ptr
  struct FileCloser {
    void operator()(std::FILE * archivo) const noexcept {
      if (archivo != nullptr) {
        // NOLINTNEXTLINE(cert-err33-c,cppcoreguidelines-owning-memory)
        (void) std::fclose(archivo);
      }
    }
  };

  // alias de tipo para unique_ptr que maneja FILE* con RAII
  using FilePtr = std::unique_ptr<std::FILE, FileCloser>;

  // abre archivo para escritura con manejo RAII
  [[nodiscard]] FilePtr abrir_archivo(std::string const & ruta) {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    std::FILE * archivo = std::fopen(ruta.c_str(), "w");
    if (archivo == nullptr) {
      throw std::runtime_error(std::string("fopen fallo: ") + std::strerror(errno));
    }
    return FilePtr(archivo);
  }

  // escribe encabezado PPM con formato P3
  void escribir_encabezado(std::FILE * archivo, int ancho, int alto) {
    std::println(archivo, "P3\n{} {}\n255", ancho, alto);
  }

  // escribe triplete RGB como texto en formato PPM
  void escribir_triplete(std::FILE * archivo, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    std::println(archivo, "{} {} {}", static_cast<unsigned>(r), static_cast<unsigned>(g),
                 static_cast<unsigned>(b));
  }

}  // namespace

// escribe framebuffer AOS a archivo PPM
bool writePPM_AOS(std::string const & ruta, std::vector<Pixel> const & fb, int ancho, int alto) {
  FilePtr archivo = abrir_archivo(ruta);
  escribir_encabezado(archivo.get(), ancho, alto);

  for (int fila = 0; fila < alto; ++fila) {
    auto const indice_fila = static_cast<std::size_t>(fila) * static_cast<std::size_t>(ancho);
    for (int col = 0; col < ancho; ++col) {
      Pixel const & pixel = fb[indice_fila + static_cast<std::size_t>(col)];
      escribir_triplete(archivo.get(), pixel.r, pixel.g, pixel.b);
    }
  }
  return true;
}

// escribe framebuffer SOA a archivo PPM
bool writePPM_SOA(std::string const & ruta, FramebufferSOA const & fb, int ancho, int alto) {
  FilePtr archivo = abrir_archivo(ruta);
  escribir_encabezado(archivo.get(), ancho, alto);

  for (int fila = 0; fila < alto; ++fila) {
    auto const indice_fila = static_cast<std::size_t>(fila) * static_cast<std::size_t>(ancho);
    for (int col = 0; col < ancho; ++col) {
      auto const idx = indice_fila + static_cast<std::size_t>(col);
      escribir_triplete(archivo.get(), fb.R[idx], fb.G[idx], fb.B[idx]);
    }
  }
  return true;
}
