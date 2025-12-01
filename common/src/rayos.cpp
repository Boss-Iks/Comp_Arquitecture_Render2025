#include "../include/rayos.hpp"
#include "../include/camera.hpp"
#include "../include/scene.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace {

  constexpr double EPSILON_MAGNITUD     = 1e-12;
  constexpr double EPSILON_INTERSECCION = 1e-3;
  constexpr double EPSILON_DENOMINADOR  = 1e-8;
  constexpr double VALOR_MIN_COLOR      = 0.0;
  constexpr double VALOR_MAX_COLOR      = 255.0;
  constexpr double FACTOR_GRADIENTE_R   = 0.5;
  constexpr double FACTOR_GRADIENTE_G   = 0.3;
  constexpr double COLOR_BLANCO         = 1.0;
  constexpr double COLOR_NEGRO          = 0.0;
  constexpr double COLOR_MAGENTA_R      = 1.0;
  constexpr double COLOR_MAGENTA_G      = 0.0;
  constexpr double COLOR_MAGENTA_B      = 1.0;
  constexpr double COEF_CUADRATICA      = 2.0;
  constexpr double COEF_CUADRATICA_INV  = 4.0;
  constexpr double NEGATIVO             = -1.0;

  [[nodiscard]] inline std::array<double, 3> normalize(std::array<double, 3> const & a) {
    double const magnitud = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (magnitud > EPSILON_MAGNITUD) {
      return {a[0] / magnitud, a[1] / magnitud, a[2] / magnitud};
    }
    return {COLOR_NEGRO, COLOR_NEGRO, COLOR_NEGRO};
  }

  [[nodiscard]] inline double dot(std::array<double, 3> const & a,
                                  std::array<double, 3> const & b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  [[nodiscard]] inline std::array<double, 3> sub(std::array<double, 3> const & a,
                                                 std::array<double, 3> const & b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  [[nodiscard]] inline std::array<double, 3> add(std::array<double, 3> const & a,
                                                 std::array<double, 3> const & b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  [[nodiscard]] inline std::array<double, 3> mul(std::array<double, 3> const & a, double escalar) {
    return {a[0] * escalar, a[1] * escalar, a[2] * escalar};
  }

  [[nodiscard]] inline double length(std::array<double, 3> const & a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  }

  [[nodiscard]] inline std::array<double, 3> perp_to_axis(std::array<double, 3> const & v,
                                                          std::array<double, 3> const & a) {
    double const proyeccion = dot(v, a);
    return sub(v, mul(a, proyeccion));
  }

  [[nodiscard]] inline std::uint8_t color_a_byte(double valor) {
    double const clamped = std::clamp(valor * VALOR_MAX_COLOR, VALOR_MIN_COLOR, VALOR_MAX_COLOR);
    return static_cast<std::uint8_t>(clamped);
  }

  [[nodiscard]] inline Pixel calcular_color_fondo(std::size_t fila, std::size_t alto_total) {
    double const t = static_cast<double>(fila) / static_cast<double>(alto_total);
    return {color_a_byte(COLOR_BLANCO - t * FACTOR_GRADIENTE_R),
            color_a_byte(COLOR_BLANCO - t * FACTOR_GRADIENTE_G), color_a_byte(COLOR_BLANCO)};
  }

  [[nodiscard]] inline bool validar_distancia(double distancia, double t_actual) {
    return distancia >= EPSILON_INTERSECCION and distancia < t_actual;
  }

  bool intersectar_esfera(Ray const & rayo, Sphere const & esfera, HitRecord & hit) {
    auto const rc = sub(esfera.center, rayo.origin);

    double const a             = dot(rayo.direction, rayo.direction);
    double const b             = -COEF_CUADRATICA * dot(rayo.direction, rc);
    double const c             = dot(rc, rc) - esfera.radius * esfera.radius;
    double const discriminante = b * b - COEF_CUADRATICA_INV * a * c;

    if (discriminante < COLOR_NEGRO) {
      return false;
    }

    double const dist = (-b - std::sqrt(discriminante)) / (COEF_CUADRATICA * a);
    if (not validar_distancia(dist, hit.t)) {
      return false;
    }

    hit.hit         = true;
    hit.t           = dist;
    hit.point       = add(rayo.origin, mul(rayo.direction, dist));
    hit.normal      = normalize(sub(hit.point, esfera.center));
    hit.material_id = esfera.material_id;
    return true;
  }

  struct DatosCilindro {
    std::array<double, 3> centro;
    std::array<double, 3> eje;
    double altura;
    double radio;
    std::uint32_t mat_id;
  };

  [[nodiscard]] DatosCilindro preparar_cilindro(Cylinder const & cilindro) {
    DatosCilindro datos{};
    datos.altura = length(cilindro.axis);
    datos.eje    = normalize(cilindro.axis);
    datos.centro = cilindro.base_center;
    datos.radio  = cilindro.radius;
    datos.mat_id = cilindro.material_id;
    return datos;
  }

  bool probar_superficie_curva(Ray const & rayo, DatosCilindro const & datos, HitRecord & hit) {
    auto const rc = sub(rayo.origin, datos.centro);
    auto const op = perp_to_axis(rc, datos.eje);
    auto const dp = perp_to_axis(rayo.direction, datos.eje);

    double const a    = dot(dp, dp);
    double const b    = COEF_CUADRATICA * dot(op, dp);
    double const c    = dot(op, op) - datos.radio * datos.radio;
    double const disc = b * b - COEF_CUADRATICA_INV * a * c;

    if (disc < COLOR_NEGRO or std::abs(a) <= EPSILON_DENOMINADOR) {
      return false;
    }

    double const dist = (-b - std::sqrt(disc)) / (COEF_CUADRATICA * a);
    if (not validar_distancia(dist, hit.t)) {
      return false;
    }

    auto const punto  = add(rayo.origin, mul(rayo.direction, dist));
    auto const ic     = sub(punto, datos.centro);
    double const proy = dot(ic, datos.eje);

    double const mitad_altura = datos.altura / COEF_CUADRATICA;
    if (proy < -mitad_altura or proy > mitad_altura) {
      return false;
    }

    hit.hit         = true;
    hit.t           = dist;
    hit.point       = punto;
    hit.normal      = normalize(perp_to_axis(ic, datos.eje));
    hit.material_id = datos.mat_id;
    return true;
  }

  struct DatosTapa {
    std::array<double, 3> centro;
    std::array<double, 3> normal;
    double radio;
    std::uint32_t mat_id;
  };

  bool probar_tapa(Ray const & rayo, DatosTapa const & tapa, HitRecord & hit) {
    double const denom = dot(rayo.direction, tapa.normal);
    if (std::abs(denom) <= EPSILON_DENOMINADOR) {
      return false;
    }

    auto const pr     = sub(tapa.centro, rayo.origin);
    double const dist = dot(pr, tapa.normal) / denom;
    if (not validar_distancia(dist, hit.t)) {
      return false;
    }

    auto const punto = add(rayo.origin, mul(rayo.direction, dist));
    auto const dr    = sub(punto, tapa.centro);
    if (length(dr) > tapa.radio) {
      return false;
    }

    hit.hit         = true;
    hit.t           = dist;
    hit.point       = punto;
    hit.normal      = tapa.normal;
    hit.material_id = tapa.mat_id;
    return true;
  }

  bool intersectar_cilindro(Ray const & rayo, Cylinder const & cilindro, HitRecord & hit) {
    auto const datos = preparar_cilindro(cilindro);
    probar_superficie_curva(rayo, datos, hit);

    auto const mitad_eje = mul(cilindro.axis, COLOR_BLANCO / COEF_CUADRATICA);
    auto const base_inf  = sub(datos.centro, mitad_eje);
    auto const base_sup  = add(datos.centro, mitad_eje);

    DatosTapa tapa_inf{base_inf, mul(datos.eje, NEGATIVO), datos.radio, datos.mat_id};
    probar_tapa(rayo, tapa_inf, hit);

    DatosTapa tapa_sup{base_sup, datos.eje, datos.radio, datos.mat_id};
    probar_tapa(rayo, tapa_sup, hit);

    return hit.hit;
  }

  [[nodiscard]] std::array<double, 3> obtener_color_material(Material const & mat) {
    switch (mat.type) {
      case MaterialType::Matte:      return mat.matte.rgb;
      case MaterialType::Metal:      return mat.metal.rgb;
      case MaterialType::Refractive: return {COLOR_BLANCO, COLOR_BLANCO, COLOR_BLANCO};
      default:                       return {COLOR_MAGENTA_R, COLOR_MAGENTA_G, COLOR_MAGENTA_B};
    }
  }

  [[nodiscard]] std::array<double, 3> calcular_pos_pixel(Camera const & cam, std::size_t col,
                                                         std::size_t fila) {
    auto const cd = static_cast<double>(col);
    auto const fd = static_cast<double>(fila);
    return {cam.O[0] + cd * cam.dx[0] + fd * cam.dy[0], cam.O[1] + cd * cam.dx[1] + fd * cam.dy[1],
            cam.O[2] + cd * cam.dx[2] + fd * cam.dy[2]};
  }

  void buscar_intersecciones(Ray const & rayo, Scene const & escena, HitRecord & hit) {
    for (auto const & esfera : escena.spheres) {
      intersectar_esfera(rayo, esfera, hit);
    }
    for (auto const & cilindro : escena.cylinders) {
      intersectar_cilindro(rayo, cilindro, hit);
    }
  }

  [[nodiscard]] Pixel calcular_color(HitRecord const & hit, Scene const & escena, std::size_t fila,
                                     std::size_t alto) {
    if (not hit.hit) {
      return calcular_color_fondo(fila, alto);
    }

    auto const & mat = escena.materials[hit.material_id];
    auto const color = obtener_color_material(mat);
    return {color_a_byte(color[0]), color_a_byte(color[1]), color_a_byte(color[2])};
  }

  [[nodiscard]] Ray crear_rayo(Camera const & cam, std::array<double, 3> const & pos) {
    Ray rayo{};
    rayo.origin    = cam.P;
    rayo.direction = normalize(sub(pos, cam.P));
    return rayo;
  }

}  // namespace

void trace_rays_aos(Camera const & camara, Scene const & escena, std::vector<Pixel> & framebuffer) {
  auto const ancho = static_cast<std::size_t>(camara.image_width);
  auto const alto  = static_cast<std::size_t>(camara.image_height);
  framebuffer.resize(ancho * alto);

  for (std::size_t fila = 0; fila < alto; ++fila) {
    for (std::size_t col = 0; col < ancho; ++col) {
      auto const pos  = calcular_pos_pixel(camara, col, fila);
      auto const rayo = crear_rayo(camara, pos);
      HitRecord hit{};
      hit.t = std::numeric_limits<double>::infinity();
      buscar_intersecciones(rayo, escena, hit);
      framebuffer[fila * ancho + col] = calcular_color(hit, escena, fila, alto);
    }
  }
}

void trace_rays_soa(Camera const & camara, Scene const & escena, FramebufferSOA & framebuffer) {
  auto const ancho = static_cast<std::size_t>(camara.image_width);
  auto const alto  = static_cast<std::size_t>(camara.image_height);
  framebuffer.R.resize(ancho * alto);
  framebuffer.G.resize(ancho * alto);
  framebuffer.B.resize(ancho * alto);

  for (std::size_t fila = 0; fila < alto; ++fila) {
    for (std::size_t col = 0; col < ancho; ++col) {
      auto const pos  = calcular_pos_pixel(camara, col, fila);
      auto const rayo = crear_rayo(camara, pos);
      HitRecord hit{};
      hit.t = std::numeric_limits<double>::infinity();
      buscar_intersecciones(rayo, escena, hit);
      auto const pixel   = calcular_color(hit, escena, fila, alto);
      auto const idx     = fila * ancho + col;
      framebuffer.R[idx] = pixel.r;
      framebuffer.G[idx] = pixel.g;
      framebuffer.B[idx] = pixel.b;
    }
  }
}
