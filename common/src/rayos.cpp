#include "../include/rayos.hpp"
#include "../include/camera.hpp"
#include "../include/scene.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>

namespace {

  constexpr double EPSILON_MAGNITUD     = 1e-12;
  constexpr double EPSILON_INTERSECCION = 1e-3;
  constexpr double EPSILON_DENOMINADOR  = 1e-8;
  constexpr double VALOR_MIN_COLOR      = 0.0;
  constexpr double VALOR_MAX_COLOR      = 255.0;
  constexpr double COLOR_BLANCO         = 1.0;
  constexpr double COLOR_NEGRO          = 0.0;
  constexpr double COEF_CUADRATICA      = 2.0;
  constexpr double COEF_CUADRATICA_INV  = 4.0;
  constexpr double NEGATIVO             = -1.0;
  constexpr double VECTOR_PEQUENYO      = 1e-8;

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

  [[nodiscard]] inline bool vector_demasiado_pequenyo(std::array<double, 3> const & v) {
    return std::abs(v[0]) < VECTOR_PEQUENYO and
           std::abs(v[1]) < VECTOR_PEQUENYO and
           std::abs(v[2]) < VECTOR_PEQUENYO;
  }

  [[nodiscard]] inline std::array<double, 3> calcular_color_fondo(
      std::array<double, 3> const & direccion, Camera const & cam) {
    auto const dir_unit = normalize(direccion);
    double const mezcla = (dir_unit[1] + COLOR_BLANCO) / COEF_CUADRATICA;

    auto const & cl = cam.bg_light;
    auto const & cd = cam.bg_dark;

    return {(COLOR_BLANCO - mezcla) * cl[0] + mezcla * cd[0],
            (COLOR_BLANCO - mezcla) * cl[1] + mezcla * cd[1],
            (COLOR_BLANCO - mezcla) * cl[2] + mezcla * cd[2]};
  }

  [[nodiscard]] inline bool validar_distancia(double distancia, double t_actual) {
    return distancia >= EPSILON_INTERSECCION and distancia < t_actual;
  }

  bool intersectar_esfera(Ray const & rayo, Sphere const & esfera, HitRecord & hit) {
    auto const rc              = sub(esfera.center, rayo.origin);
    double const a             = dot(rayo.direction, rayo.direction);
    double const b             = -COEF_CUADRATICA * dot(rayo.direction, rc);
    double const c             = dot(rc, rc) - esfera.radius * esfera.radius;
    double const discriminante = b * b - COEF_CUADRATICA_INV * a * c;
    if (discriminante < COLOR_NEGRO) {
      return false;
    }
    double const raiz      = std::sqrt(discriminante);
    double const inv_dos_a = 1.0 / (COEF_CUADRATICA * a);
    double const d1        = (-b - raiz) * inv_dos_a;
    double const d2        = (-b + raiz) * inv_dos_a;
    bool encontrado        = false;
    double distancia       = hit.t;
    if (validar_distancia(d1, distancia)) {
      distancia  = d1;
      encontrado = true;
    }
    if (validar_distancia(d2, distancia)) {
      distancia  = d2;
      encontrado = true;
    }
    if (not encontrado) {
      return false;
    }
    hit.hit    = true;
    hit.t      = distancia;
    hit.point  = add(rayo.origin, mul(rayo.direction, distancia));
    hit.normal = normalize(sub(hit.point, esfera.center));
    if (dot(rayo.direction, hit.normal) > 0.0) {
      hit.normal = mul(hit.normal, -1.0);
    }
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
    auto const rc     = sub(rayo.origin, datos.centro);
    auto const op     = perp_to_axis(rc, datos.eje);
    auto const dp     = perp_to_axis(rayo.direction, datos.eje);
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
    auto const punto          = add(rayo.origin, mul(rayo.direction, dist));
    auto const ic             = sub(punto, datos.centro);
    double const proy         = dot(ic, datos.eje);
    double const mitad_altura = datos.altura / COEF_CUADRATICA;
    if (proy < -mitad_altura or proy > mitad_altura) {
      return false;
    }

    hit.hit    = true;
    hit.t      = dist;
    hit.point  = punto;
    hit.normal = normalize(perp_to_axis(ic, datos.eje));
    if (dot(rayo.direction, hit.normal) > 0.0) {
      hit.normal = mul(hit.normal, -1.0);
    }
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

    hit.hit    = true;
    hit.t      = dist;
    hit.point  = punto;
    hit.normal = tapa.normal;
    if (dot(rayo.direction, hit.normal) > 0.0) {
      hit.normal = mul(hit.normal, -1.0);
    }
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

  void buscar_intersecciones(Ray const & rayo, Scene const & escena, HitRecord & hit) {
    for (auto const & esfera : escena.spheres) {
      intersectar_esfera(rayo, esfera, hit);
    }
    for (auto const & cilindro : escena.cylinders) {
      intersectar_cilindro(rayo, cilindro, hit);
    }
  }

  [[nodiscard]] inline std::array<double, 3> calcular_pos_pixel(Camera const & cam, double col,
                                                                double fila) {
    return {cam.O[0] + col * cam.dx[0] + fila * cam.dy[0],
            cam.O[1] + col * cam.dx[1] + fila * cam.dy[1],
            cam.O[2] + col * cam.dx[2] + fila * cam.dy[2]};
  }

  struct RayContext {
    std::size_t depth;
    std::mt19937_64 * material_rng;
  };

  struct ReflectionResult {
    std::array<double, 3> direction;
    std::array<double, 3> reflectancia;
  };

  [[nodiscard]] ReflectionResult calcular_reflexion_mate(std::array<double, 3> const & normal,
                                                         Material const & mat,
                                                         std::mt19937_64 * rng) {
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::array<double, 3> dr{normal[0] + dist(*rng), normal[1] + dist(*rng),
                             normal[2] + dist(*rng)};
    if (vector_demasiado_pequenyo(dr)) {
      dr = normal;
    }
    return {normalize(dr), mat.matte.rgb};
  }

  [[nodiscard]] ReflectionResult calcular_reflexion_metal(std::array<double, 3> const & d_hat,
                                                          std::array<double, 3> const & normal,
                                                          Material const & mat,
                                                          std::mt19937_64 * rng) {
    auto const d1     = sub(d_hat, mul(normal, 2.0 * dot(d_hat, normal)));
    auto const d1_hat = normalize(d1);

    std::uniform_real_distribution<double> dist(-mat.metal.diffusion, mat.metal.diffusion);
    std::array<double, 3> ruido{dist(*rng), dist(*rng), dist(*rng)};

    auto const dr_final = add(d1_hat, ruido);

    return {dr_final, mat.metal.rgb};
  }

  // FIXED: Corrected refractive index calculation per specification section 3.5.3
  // If outward: ρ' = 1/η
  // If inward: ρ' = η
  [[nodiscard]] ReflectionResult calcular_reflexion_refractiva(std::array<double, 3> const & d_hat,
                                                               std::array<double, 3> normal,
                                                               Material const & mat) {
    bool const hacia_fuera = dot(d_hat, normal) < 0.0;
    double const cos_theta = std::min(-dot(d_hat, normal), 1.0);
    double const sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));

    // CORRECTED: outward = 1/η, inward = η
    double rho_p = hacia_fuera ? (1.0 / mat.refr.index) : mat.refr.index;

    if (not hacia_fuera) {
      normal = mul(normal, -1.0);
    }

    std::array<double, 3> dr{};
    if (rho_p * sin_theta > 1.0) {
      dr = sub(d_hat, mul(normal, 2.0 * dot(d_hat, normal)));
    } else {
      auto const u = mul(add(d_hat, mul(normal, cos_theta)), rho_p);
      auto const v = mul(normal, -std::sqrt(std::max(0.0, 1.0 - dot(u, u))));
      dr           = add(u, v);
    }
    return {
      normalize(dr), {1.0, 1.0, 1.0}
    };
  }

  [[nodiscard]] std::array<double, 3> ray_color(Ray const & rayo, Scene const * escena,
                                                Camera const * cam, RayContext & ctx);

  [[nodiscard]] ReflectionResult calcular_reflexion(std::array<double, 3> const & d_hat,
                                                    std::array<double, 3> const & normal,
                                                    Material const & mat, std::mt19937_64 * rng) {
    switch (mat.type) {
      case MaterialType::Matte:      return calcular_reflexion_mate(normal, mat, rng);
      case MaterialType::Metal:      return calcular_reflexion_metal(d_hat, normal, mat, rng);
      case MaterialType::Refractive: return calcular_reflexion_refractiva(d_hat, normal, mat);
      default:
        return {
          {1.0, 0.0, 1.0},
          {1.0, 0.0, 1.0}
        };
    }
  }

  [[nodiscard]] std::array<double, 3> ray_color(Ray const & rayo, Scene const * escena,
                                                Camera const * cam, RayContext & ctx) {
    if (ctx.depth == 0U) {
      return {0.0, 0.0, 0.0};
    }

    HitRecord hit{};
    hit.t   = std::numeric_limits<double>::infinity();
    hit.hit = false;
    buscar_intersecciones(rayo, *escena, hit);

    if (not hit.hit) {
      return calcular_color_fondo(rayo.direction, *cam);
    }

    auto const & mat = escena->materials[hit.material_id];
    auto const d_hat = normalize(rayo.direction);
    auto const refl  = calcular_reflexion(d_hat, hit.normal, mat, ctx.material_rng);

    Ray siguiente{};
    siguiente.origin    = hit.point;
    siguiente.direction = refl.direction;

    ctx.depth -= 1U;
    auto const c_sig = ray_color(siguiente, escena, cam, ctx);
    ctx.depth += 1U;

    return {c_sig[0] * refl.reflectancia[0], c_sig[1] * refl.reflectancia[1],
            c_sig[2] * refl.reflectancia[2]};
  }

  [[nodiscard]] inline Pixel color_a_pixel(std::array<double, 3> const & c, double gamma) {
    auto corregir = [gamma](double v) {
      v = std::clamp(v, 0.0, 1.0);
      if (gamma > 0.0) {
        v = std::pow(v, 1.0 / gamma);
      }
      return v;
    };
    return {color_a_byte(corregir(c[0])), color_a_byte(corregir(c[1])),
            color_a_byte(corregir(c[2]))};
  }

}  // namespace

void trace_rays_aos(Camera const & camara, Scene const & escena, std::vector<Pixel> & framebuffer) {
  auto const ancho = static_cast<std::size_t>(camara.image_width);
  auto const alto  = static_cast<std::size_t>(camara.image_height);
  framebuffer.resize(ancho * alto);
  std::mt19937_64 generador_material{
    static_cast<std::mt19937_64::result_type>(camara.material_rng_seed)};
  std::mt19937_64 generador_rayos{static_cast<std::mt19937_64::result_type>(camara.ray_rng_seed)};
  std::uniform_real_distribution<double> aleatoriedad(-0.5, 0.5);
  auto const spp       = static_cast<std::size_t>(camara.samples_per_pixel);
  auto const max_depth = static_cast<std::size_t>(camara.max_depth);
  for (std::size_t fila = 0; fila < alto; ++fila) {
    for (std::size_t col = 0; col < ancho; ++col) {
      std::array<double, 3> acumulado{0.0, 0.0, 0.0};
      for (std::size_t s = 0; s < spp; ++s) {
        auto const pos =
            calcular_pos_pixel(camara, static_cast<double>(col) + aleatoriedad(generador_rayos),
                               static_cast<double>(fila) + aleatoriedad(generador_rayos));
        Ray rayo{};
        rayo.origin    = camara.P;
        rayo.direction = normalize(sub(pos, camara.P));
        RayContext ctx{max_depth, &generador_material};
        auto const c = ray_color(rayo, &escena, &camara, ctx);
        acumulado[0] += c[0];
        acumulado[1] += c[1];
        acumulado[2] += c[2];
      }
      acumulado[0] /= static_cast<double>(spp);
      acumulado[1] /= static_cast<double>(spp);
      acumulado[2] /= static_cast<double>(spp);
      framebuffer[fila * ancho + col] = color_a_pixel(acumulado, camara.gamma);
    }
  }
}

void trace_rays_soa(Camera const & camara, Scene const & escena, FramebufferSOA & framebuffer) {
  auto const ancho = static_cast<std::size_t>(camara.image_width);
  auto const alto  = static_cast<std::size_t>(camara.image_height);
  framebuffer.R.resize(ancho * alto);
  framebuffer.G.resize(ancho * alto);
  framebuffer.B.resize(ancho * alto);
  std::mt19937_64 generador_material{
    static_cast<std::mt19937_64::result_type>(camara.material_rng_seed)};
  std::mt19937_64 generador_rayos{static_cast<std::mt19937_64::result_type>(camara.ray_rng_seed)};
  std::uniform_real_distribution<double> aleatoriedad(-0.5, 0.5);
  auto const spp       = static_cast<std::size_t>(camara.samples_per_pixel);
  auto const max_depth = static_cast<std::size_t>(camara.max_depth);
  for (std::size_t fila = 0; fila < alto; ++fila) {
    for (std::size_t col = 0; col < ancho; ++col) {
      std::array<double, 3> acumulado{0.0, 0.0, 0.0};
      for (std::size_t s = 0; s < spp; ++s) {
        auto const pos =
            calcular_pos_pixel(camara, static_cast<double>(col) + aleatoriedad(generador_rayos),
                               static_cast<double>(fila) + aleatoriedad(generador_rayos));
        Ray rayo{};
        rayo.origin    = camara.P;
        rayo.direction = normalize(sub(pos, camara.P));
        RayContext ctx{max_depth, &generador_material};
        auto const c = ray_color(rayo, &escena, &camara, ctx);
        acumulado[0] += c[0];
        acumulado[1] += c[1];
        acumulado[2] += c[2];
      }
      acumulado[0] /= static_cast<double>(spp);
      acumulado[1] /= static_cast<double>(spp);
      acumulado[2] /= static_cast<double>(spp);
      auto const px      = color_a_pixel(acumulado, camara.gamma);
      auto const idx     = fila * ancho + col;
      framebuffer.R[idx] = px.r;
      framebuffer.G[idx] = px.g;
      framebuffer.B[idx] = px.b;
    }
  }
}
