#include "../include/rayos.hpp"
#include "../include/camera.hpp"
#include "../include/scene.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

  // constantes para tolerancias numericas y limites de color
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

  // normaliza un vector 3D a longitud unitaria
  [[nodiscard]] inline std::array<double, 3> normalize(std::array<double, 3> const & a) {
    double const magnitud = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (magnitud > EPSILON_MAGNITUD) {
      return {a[0] / magnitud, a[1] / magnitud, a[2] / magnitud};
    }
    return {COLOR_NEGRO, COLOR_NEGRO, COLOR_NEGRO};
  }

  // calcula producto punto entre dos vectores 3D
  [[nodiscard]] inline double dot(std::array<double, 3> const & a,
                                  std::array<double, 3> const & b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  // resta dos vectores 3D componente a componente
  [[nodiscard]] inline std::array<double, 3> sub(std::array<double, 3> const & a,
                                                 std::array<double, 3> const & b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  // suma dos vectores 3D componente a componente
  [[nodiscard]] inline std::array<double, 3> add(std::array<double, 3> const & a,
                                                 std::array<double, 3> const & b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  // multiplica un vector 3D por un escalar
  [[nodiscard]] inline std::array<double, 3> mul(std::array<double, 3> const & a, double escalar) {
    return {a[0] * escalar, a[1] * escalar, a[2] * escalar};
  }

  // calcula la magnitud (longitud) de un vector 3D
  [[nodiscard]] inline double length(std::array<double, 3> const & a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  }

  // proyecta vector v sobre el plano perpendicular al eje a
  [[nodiscard]] inline std::array<double, 3> perp_to_axis(std::array<double, 3> const & v,
                                                          std::array<double, 3> const & a) {
    double const proyeccion = dot(v, a);
    return sub(v, mul(a, proyeccion));
  }

  // convierte valor en rango [0,1] a byte [0,255] con clamp
  [[nodiscard]] inline std::uint8_t color_a_byte(double valor) {
    double const clamped = std::clamp(valor * VALOR_MAX_COLOR, VALOR_MIN_COLOR, VALOR_MAX_COLOR);
    return static_cast<std::uint8_t>(clamped);
  }

  // genera color de fondo como gradiente vertical de azul claro a blanco
  [[nodiscard]] inline Pixel calcular_color_fondo(std::size_t fila, std::size_t alto_total) {
    double const t = static_cast<double>(fila) / static_cast<double>(alto_total);
    return {color_a_byte(COLOR_BLANCO - t * FACTOR_GRADIENTE_R),
            color_a_byte(COLOR_BLANCO - t * FACTOR_GRADIENTE_G), color_a_byte(COLOR_BLANCO)};
  }

  // verifica si la distancia de interseccion es valida (no muy cerca ni muy lejos)
  [[nodiscard]] inline bool validar_distancia(double distancia, double t_actual) {
    return distancia >= EPSILON_INTERSECCION and distancia < t_actual;
  }

  // calcula interseccion rayo-esfera usando ecuacion cuadratica
  bool intersectar_esfera(Ray const & rayo, Sphere const & esfera, HitRecord & hit) {
    // vector del origen del rayo al centro de la esfera
    auto const rc = sub(esfera.center, rayo.origin);

    // coeficientes de ecuacion cuadratica: at^2 + bt + c = 0
    double const a             = dot(rayo.direction, rayo.direction);
    double const b             = -COEF_CUADRATICA * dot(rayo.direction, rc);
    double const c             = dot(rc, rc) - esfera.radius * esfera.radius;
    double const discriminante = b * b - COEF_CUADRATICA_INV * a * c;

    // sin solucion real si discriminante es negativo
    if (discriminante < COLOR_NEGRO) {
      return false;
    }

    // tomar la solucion mas cercana (raiz negativa)
    double const dist = (-b - std::sqrt(discriminante)) / (COEF_CUADRATICA * a);
    if (not validar_distancia(dist, hit.t)) {
      return false;
    }

    // actualizar registro de hit con datos de interseccion
    hit.hit         = true;
    hit.t           = dist;
    hit.point       = add(rayo.origin, mul(rayo.direction, dist));
    hit.normal      = normalize(sub(hit.point, esfera.center));
    hit.material_id = esfera.material_id;
    return true;
  }

  // estructura para datos procesados del cilindro
  struct DatosCilindro {
    std::array<double, 3> eje;
    std::array<double, 3> desp_origen;
    double altura;
    double radio;
    std::uint32_t mat_id;
  };

  // prepara datos del cilindro para calculo de interseccion
  [[nodiscard]] DatosCilindro preparar_cilindro(Ray const & rayo, Cylinder const & cilindro) {
    DatosCilindro datos{};
    datos.altura      = length(cilindro.axis);
    datos.eje         = normalize(cilindro.axis);
    datos.desp_origen = sub(rayo.origin, cilindro.base_center);
    datos.radio       = cilindro.radius;
    datos.mat_id      = cilindro.material_id;
    return datos;
  }

  // prueba interseccion con superficie curva del cilindro (cilindro infinito)
  bool probar_superficie_curva(Ray const & rayo, DatosCilindro const & datos, HitRecord & hit) {
    // proyectar origen y direccion del rayo al plano perpendicular al eje
    auto const op = perp_to_axis(datos.desp_origen, datos.eje);
    auto const dp = perp_to_axis(rayo.direction, datos.eje);

    // resolver ecuacion cuadratica en el plano perpendicular
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

    // verificar que el punto este dentro de los limites de altura
    auto const punto  = add(rayo.origin, mul(rayo.direction, dist));
    auto const desp   = sub(punto, sub(rayo.origin, datos.desp_origen));
    double const proy = dot(desp, datos.eje);

    if (proy < COLOR_NEGRO or proy > datos.altura) {
      return false;
    }

    // calcular normal perpendicular al eje
    hit.hit         = true;
    hit.t           = dist;
    hit.point       = punto;
    hit.normal      = normalize(perp_to_axis(desp, datos.eje));
    hit.material_id = datos.mat_id;
    return true;
  }

  // estructura para datos de una tapa circular del cilindro
  struct DatosTapa {
    std::array<double, 3> centro;
    std::array<double, 3> normal;
    double radio;
    std::uint32_t mat_id;
  };

  // prueba interseccion con tapa circular del cilindro (disco plano)
  bool probar_tapa(Ray const & rayo, DatosTapa const & tapa, HitRecord & hit) {
    // interseccion rayo-plano
    double const denom = dot(rayo.direction, tapa.normal);
    if (std::abs(denom) <= EPSILON_DENOMINADOR) {
      return false;
    }

    auto const pr     = sub(tapa.centro, rayo.origin);
    double const dist = dot(pr, tapa.normal) / denom;
    if (not validar_distancia(dist, hit.t)) {
      return false;
    }

    // verificar que el punto este dentro del circulo
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

  // calcula interseccion rayo-cilindro probando superficie curva y ambas tapas
  bool intersectar_cilindro(Ray const & rayo, Cylinder const & cilindro, HitRecord & hit) {
    auto const datos = preparar_cilindro(rayo, cilindro);
    probar_superficie_curva(rayo, datos, hit);

    // probar tapa inferior
    DatosTapa tapa_inf{cilindro.base_center, mul(datos.eje, NEGATIVO), datos.radio, datos.mat_id};
    probar_tapa(rayo, tapa_inf, hit);

    // probar tapa superior
    DatosTapa tapa_sup{add(cilindro.base_center, cilindro.axis), datos.eje, datos.radio,
                       datos.mat_id};
    probar_tapa(rayo, tapa_sup, hit);

    return hit.hit;
  }

  // extrae color RGB del material segun su tipo (mate, metal, refractivo)
  [[nodiscard]] std::array<double, 3> obtener_color_material(Material const & mat) {
    switch (mat.type) {
      case MaterialType::Matte:      return mat.matte.rgb;
      case MaterialType::Metal:      return mat.metal.rgb;
      case MaterialType::Refractive: return {COLOR_BLANCO, COLOR_BLANCO, COLOR_BLANCO};
      default:                       return {COLOR_MAGENTA_R, COLOR_MAGENTA_G, COLOR_MAGENTA_B};
    }
  }

  // calcula posicion 3D en el plano de proyeccion para coordenadas de pixel
  [[nodiscard]] std::array<double, 3> calcular_pos_pixel(Camera const & cam, std::size_t col,
                                                         std::size_t fila) {
    auto const cd = static_cast<double>(col);
    auto const fd = static_cast<double>(fila);
    return {cam.O[0] + cd * cam.dx[0] + fd * cam.dy[0], cam.O[1] + cd * cam.dx[1] + fd * cam.dy[1],
            cam.O[2] + cd * cam.dx[2] + fd * cam.dy[2]};
  }

  // busca interseccion mas cercana del rayo con toda la geometria de la escena
  void buscar_intersecciones(Ray const & rayo, Scene const & escena, HitRecord & hit) {
    for (auto const & esfera : escena.spheres) {
      intersectar_esfera(rayo, esfera, hit);
    }
    for (auto const & cilindro : escena.cylinders) {
      intersectar_cilindro(rayo, cilindro, hit);
    }
  }

  // calcula color final del pixel basado en hit o fondo
  [[nodiscard]] Pixel calcular_color(HitRecord const & hit, Scene const & escena, std::size_t fila,
                                     std::size_t alto) {
    if (not hit.hit) {
      return calcular_color_fondo(fila, alto);
    }

    auto const & mat = escena.materials[hit.material_id];
    auto const color = obtener_color_material(mat);
    return {color_a_byte(color[0]), color_a_byte(color[1]), color_a_byte(color[2])};
  }

  // crea rayo desde camara hacia posicion en plano de proyeccion
  [[nodiscard]] Ray crear_rayo(Camera const & cam, std::array<double, 3> const & pos) {
    Ray rayo{};
    rayo.origin    = cam.P;
    rayo.direction = normalize(sub(pos, cam.P));
    return rayo;
  }

}  // namespace

// renderiza escena usando buffer AOS (array of structures)
void trace_rays_aos(Camera const & camara, Scene const & escena, std::vector<Pixel> & framebuffer) {
  auto const ancho = static_cast<std::size_t>(camara.image_width);
  auto const alto  = static_cast<std::size_t>(camara.image_height);
  framebuffer.resize(ancho * alto);

  for (std::size_t fila = 0; fila < alto; ++fila) {
    for (std::size_t col = 0; col < ancho; ++col) {
      auto const pos  = calcular_pos_pixel(camara, col, fila);
      auto const rayo = crear_rayo(camara, pos);
      HitRecord hit{};
      buscar_intersecciones(rayo, escena, hit);
      framebuffer[fila * ancho + col] = calcular_color(hit, escena, fila, alto);
    }
  }
}

// renderiza escena usando buffer SOA (structure of arrays)
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
      buscar_intersecciones(rayo, escena, hit);
      auto const pixel   = calcular_color(hit, escena, fila, alto);
      auto const idx     = fila * ancho + col;
      framebuffer.R[idx] = pixel.r;
      framebuffer.G[idx] = pixel.g;
      framebuffer.B[idx] = pixel.b;
    }
  }
}
