#include "rayos.hpp"
#include "../common/include/camera.hpp"
#include "../common/include/config.hpp"
#include "../common/include/scene.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

// Forward declarations for common types
struct CameraConfig;
struct SceneData;

// helper functions
inline std::array<float, 3> normalize(std::array<float, 3> const & a) {
  float mag = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (mag > 1e-12f) {
    return {a[0] / mag, a[1] / mag, a[2] / mag};
  } else {
    return {0.0f, 0.0f, 0.0f};
  }
}

inline std::array<double, 3> normalize_d(std::array<double, 3> const & a) {
  double mag = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (mag > 1e-12) {
    return {a[0] / mag, a[1] / mag, a[2] / mag};
  } else {
    return {0.0, 0.0, 0.0};
  }
}

inline float dot(std::array<float, 3> const & a, std::array<float, 3> const & b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline double dot_d(std::array<double, 3> const & a, std::array<double, 3> const & b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline std::array<float, 3> cross(std::array<float, 3> const & a, std::array<float, 3> const & b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

inline std::array<double, 3> sub_d(std::array<double, 3> const & a,
                                   std::array<double, 3> const & b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline std::array<double, 3> add_d(std::array<double, 3> const & a,
                                   std::array<double, 3> const & b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline std::array<double, 3> mul_d(std::array<double, 3> const & a, double s) {
  return {a[0] * s, a[1] * s, a[2] * s};
}

inline double length_d(std::array<double, 3> const & a) {
  return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// Perpendicular component to axis a
inline std::array<double, 3> perp_to_axis(std::array<double, 3> const & v,
                                          std::array<double, 3> const & a) {
  double va = dot_d(v, a);
  return sub_d(v, mul_d(a, va));
}

// generate
void generacion_rayos(CameraConfig const & camera, int image_width, int image_height,
                      std::array<float, 3> & origen_ventana, std::array<float, 3> & paso_x,
                      std::array<float, 3> & paso_y) {
  // Vector focal -> from point of view to the destination of vision
  std::array<float, 3> vf = {static_cast<float>(camera.position[0] - camera.target[0]),
                             static_cast<float>(camera.position[1] - camera.target[1]),
                             static_cast<float>(camera.position[2] - camera.target[2])};

  // Distancia focal
  float df    = std::sqrt(vf[0] * vf[0] + vf[1] * vf[1] + vf[2] * vf[2]);
  float pi    = 3.1415927f;
  float alpha = (camera.fov_deg * pi) / 180.0f;
  float hp    = 2.0f * std::tan(0.5f * alpha) * df;
  float wp    = hp * static_cast<float>(image_width) / static_cast<float>(image_height);

  // Vector up normalizado
  std::array<float, 3> up = {static_cast<float>(camera.up[0]), static_cast<float>(camera.up[1]),
                             static_cast<float>(camera.up[2])};
  up                      = normalize(up);

  // Vectores u, v, w
  std::array<float, 3> w = normalize(vf);
  std::array<float, 3> u = normalize(cross(up, w));
  std::array<float, 3> v = cross(w, u);

  // Centro de la ventana de proyección
  std::array<float, 3> centro_ventana = {static_cast<float>(camera.position[0]) - df * w[0],
                                         static_cast<float>(camera.position[1]) - df * w[1],
                                         static_cast<float>(camera.position[2]) - df * w[2]};

  // Esquina superior izquierda
  origen_ventana = {centro_ventana[0] - 0.5f * wp * u[0] + 0.5f * hp * v[0],
                    centro_ventana[1] - 0.5f * wp * u[1] + 0.5f * hp * v[1],
                    centro_ventana[2] - 0.5f * wp * u[2] + 0.5f * hp * v[2]};

  // Paso en X y Y
  paso_x = {wp * u[0] / image_width, wp * u[1] / image_width, wp * u[2] / image_width};
  paso_y = {-hp * v[0] / image_height, -hp * v[1] / image_height, -hp * v[2] / image_height};
}

static bool intersect_sphere(Ray const & ray, std::array<double, 3> const & center, double radius,
                             uint32_t mat_id, HitRecord & hit) {
  auto rc     = sub_d(center, ray.origin);
  double a    = dot_d(ray.direction, ray.direction);
  double b    = -2.0 * dot_d(ray.direction, rc);
  double c    = dot_d(rc, rc) - radius * radius;
  double disc = b * b - 4.0 * a * c;

  if (disc < 0.0) {
    return false;
  }

  double t = (-b - std::sqrt(disc)) / (2.0 * a);
  if (t < 1e-3 || t >= hit.t) {
    return false;
  }

  hit.hit         = true;
  hit.t           = t;
  hit.point       = add_d(ray.origin, mul_d(ray.direction, t));
  hit.normal      = normalize_d(sub_d(hit.point, center));
  hit.material_id = mat_id;
  return true;
}

static bool intersect_cylinder(Ray const & ray, std::array<double, 3> const & base_center,
                               double radius, std::array<double, 3> const & axis, uint32_t mat_id,
                               HitRecord & hit) {
  // Normalize axis and compute height
  double h                = length_d(axis);
  std::array<double, 3> a = normalize_d(axis);

  auto rc = sub_d(ray.origin, base_center);

  // Intersect with infinite cylinder curved surface
  auto rc_perp = perp_to_axis(rc, a);
  auto dr_perp = perp_to_axis(ray.direction, a);

  double A = dot_d(dr_perp, dr_perp);
  double B = 2.0 * dot_d(rc_perp, dr_perp);
  double C = dot_d(rc_perp, rc_perp) - radius * radius;

  double disc       = B * B - 4.0 * A * C;
  bool found_curved = false;

  if (disc >= 0.0 && std::abs(A) > 1e-8) {
    double t = (-B - std::sqrt(disc)) / (2.0 * A);
    if (t >= 1e-3 && t < hit.t) {
      auto point  = add_d(ray.origin, mul_d(ray.direction, t));
      auto ic     = sub_d(point, base_center);
      double proj = dot_d(ic, a);

      if (proj >= 0.0 && proj <= h) {
        hit.hit         = true;
        hit.t           = t;
        hit.point       = point;
        auto perp_ic    = perp_to_axis(ic, a);
        hit.normal      = normalize_d(perp_ic);
        hit.material_id = mat_id;
        found_curved    = true;
      }
    }
  }

  // Intersect with top and bottom caps
  // Bottom cap (at base_center)
  auto pn_bottom      = mul_d(a, -1.0);
  double denom_bottom = dot_d(ray.direction, pn_bottom);
  if (std::abs(denom_bottom) > 1e-8) {
    auto rp  = sub_d(base_center, ray.origin);
    double t = dot_d(rp, pn_bottom) / denom_bottom;
    if (t >= 1e-3 && t < hit.t) {
      auto point = add_d(ray.origin, mul_d(ray.direction, t));
      auto diff  = sub_d(point, base_center);
      if (length_d(diff) <= radius) {
        hit.hit         = true;
        hit.t           = t;
        hit.point       = point;
        hit.normal      = pn_bottom;
        hit.material_id = mat_id;
        found_curved    = false;
      }
    }
  }

  // Top cap (at base_center + axis)
  auto top_center  = add_d(base_center, axis);
  auto pn_top      = a;
  double denom_top = dot_d(ray.direction, pn_top);
  if (std::abs(denom_top) > 1e-8) {
    auto rp  = sub_d(top_center, ray.origin);
    double t = dot_d(rp, pn_top) / denom_top;
    if (t >= 1e-3 && t < hit.t) {
      auto point = add_d(ray.origin, mul_d(ray.direction, t));
      auto diff  = sub_d(point, top_center);
      if (length_d(diff) <= radius) {
        hit.hit         = true;
        hit.t           = t;
        hit.point       = point;
        hit.normal      = pn_top;
        hit.material_id = mat_id;
      }
    }
  }

  return hit.hit;
}

// funcion para calcular color de un rayo
// traces a ray through the scene and calculates its color contribution
// returns the RGB values in the linear color space [0,1] interval
std::array<float, 3> calcular_color_rayo(std::array<float, 3> const & origen,
                                         std::array<float, 3> const & direccion, int profundidad,
                                         Spheres & esferas, Cylinders & cilindros, Materials & mats,
                                         std::mt19937_64 & mat_rng,
                                         std::array<float, 3> const & bg_dark,
                                         std::array<float, 3> const & bg_light) {
  // si no hay profundidad, retornar negro (no more bounces)
  if (profundidad <= 0) {
    return {0.0f, 0.0f, 0.0f};
  }

  // buscar interseccion mas cercana
  float dist_min                           = 1e30f;
  int objeto_tipo                          = -1;  // -1=ninguno, 0=esfera, 1=cilindro
  int objeto_idx                           = -1;
  std::array<float, 3> punto_interseccion  = {0.0f, 0.0f, 0.0f};
  std::array<float, 3> normal_interseccion = {0.0f, 0.0f, 0.0f};
  bool sentido_afuera                      = true;

  // interseccion con esferas
  for (int i = 0; i < esferas.count; i++) {
    // rc = C - Or (vector from ray origin to sphere center)
    std::array<float, 3> rc = {esferas.centro_x[i] - origen[0], esferas.centro_y[i] - origen[1],
                               esferas.centro_z[i] - origen[2]};

    // coeficientes ecuacion cuadratica: a*lambda^2 + b*lambda + c = 0
    float a =
        direccion[0] * direccion[0] + direccion[1] * direccion[1] + direccion[2] * direccion[2];
    float b = 2.0f * (direccion[0] * rc[0] + direccion[1] * rc[1] + direccion[2] * rc[2]);
    float c = rc[0] * rc[0] + rc[1] * rc[1] + rc[2] * rc[2] - esferas.radio[i] * esferas.radio[i];
    float discriminante = b * b - 4.0f * a * c;

    if (discriminante >= 0.0f) {
      // two solutions: lambda1 (nearest), lambda2 (farthest)
      float lambda1 = (-b - std::sqrt(discriminante)) / (2.0f * a);
      float lambda2 = (-b + std::sqrt(discriminante)) / (2.0f * a);

      // pick the nearest positive intersection
      float lambda = lambda1;
      if (lambda < 1e-3f) {
        lambda = lambda2;
      }

      if (lambda >= 1e-3f && lambda < dist_min) {
        dist_min    = lambda;
        objeto_tipo = 0;
        objeto_idx  = i;

        // calcular punto de interseccion: I = Or + lambda * dr
        punto_interseccion[0] = origen[0] + direccion[0] * lambda;
        punto_interseccion[1] = origen[1] + direccion[1] * lambda;
        punto_interseccion[2] = origen[2] + direccion[2] * lambda;

        // normal = (I - C)/r, normalized
        std::array<float, 3> temp_normal = {punto_interseccion[0] - esferas.centro_x[i],
                                            punto_interseccion[1] - esferas.centro_y[i],
                                            punto_interseccion[2] - esferas.centro_z[i]};
        normal_interseccion              = normalize(temp_normal);

        // sentido: check if ray hits from outside (dot product < 0)
        float dot_dir_normal = direccion[0] * normal_interseccion[0] +
                               direccion[1] * normal_interseccion[1] +
                               direccion[2] * normal_interseccion[2];
        sentido_afuera = (dot_dir_normal < 0.0f);

        // if hitting from inside, flip normal
        if (!sentido_afuera) {
          normal_interseccion[0] = -normal_interseccion[0];
          normal_interseccion[1] = -normal_interseccion[1];
          normal_interseccion[2] = -normal_interseccion[2];
        }
      }
    }
  }

  // interseccion con cilindros
  for (int i = 0; i < cilindros.count; i++) {
    // datos basicos del cilindro
    float Cx = cilindros.centro_x[i], Cy = cilindros.centro_y[i], Cz = cilindros.centro_z[i];
    float r  = cilindros.radio[i];
    float ax = cilindros.eje_x[i], ay = cilindros.eje_y[i], az = cilindros.eje_z[i];

    // eje unitario y altura (h = magnitude of axis vector)
    std::array<float, 3> eje_vec = {ax, ay, az};
    float amag                   = std::sqrt(ax * ax + ay * ay + az * az);
    if (amag < 1e-8f) {
      continue;  // skip degenerate cylinder
    }
    std::array<float, 3> u_eje = normalize(eje_vec);
    float ux = u_eje[0], uy = u_eje[1], uz = u_eje[2], h = amag;

    // rc = Or - C (vector from cylinder center to ray origin)
    float rcx = origen[0] - Cx, rcy = origen[1] - Cy, rcz = origen[2] - Cz;

    // components perpendicular to axis (for infinite cylinder intersection)
    float ddot = direccion[0] * ux + direccion[1] * uy + direccion[2] * uz;
    float drx = direccion[0] - ddot * ux, dry = direccion[1] - ddot * uy,
          drz = direccion[2] - ddot * uz;

    float rdot = rcx * ux + rcy * uy + rcz * uz;
    float rcpx = rcx - rdot * ux, rcpy = rcy - rdot * uy, rcpz = rcz - rdot * uz;

    // quadratic for infinite cylinder lateral surface
    float qa   = drx * drx + dry * dry + drz * drz;
    float qb   = 2.0f * (rcpx * drx + rcpy * dry + rcpz * drz);
    float qc   = (rcpx * rcpx + rcpy * rcpy + rcpz * rcpz) - r * r;
    float disc = qb * qb - 4.0f * qa * qc;

    if (qa > 1e-12f && disc >= 0.0f) {
      float sdisc  = std::sqrt(disc);
      float lambda = (-qb - sdisc) / (2.0f * qa);
      if (lambda < 1e-3f) {
        lambda = (-qb + sdisc) / (2.0f * qa);
      }

      if (lambda >= 1e-3f && lambda < dist_min) {
        // intersection point on infinite cylinder
        float Ix = origen[0] + direccion[0] * lambda;
        float Iy = origen[1] + direccion[1] * lambda;
        float Iz = origen[2] + direccion[2] * lambda;

        // check if within cylinder height limits
        float ICx = Ix - Cx, ICy = Iy - Cy, ICz = Iz - Cz;
        float proj = ICx * ux + ICy * uy + ICz * uz;
        if (std::fabs(proj) <= 0.5f * h + 1e-6f) {
          // lateral normal (perpendicular to axis)
          std::array<float, 3> temp_n = {ICx - proj * ux, ICy - proj * uy, ICz - proj * uz};
          std::array<float, 3> n_norm = normalize(temp_n);
          float nx = n_norm[0], ny = n_norm[1], nz = n_norm[2];

          // check if hitting from outside
          float d        = direccion[0] * nx + direccion[1] * ny + direccion[2] * nz;
          sentido_afuera = (d < 0.0f);
          if (!sentido_afuera) {
            nx = -nx;
            ny = -ny;
            nz = -nz;
          }

          dist_min               = lambda;
          objeto_tipo            = 1;
          objeto_idx             = i;
          punto_interseccion[0]  = Ix;
          punto_interseccion[1]  = Iy;
          punto_interseccion[2]  = Iz;
          normal_interseccion[0] = nx;
          normal_interseccion[1] = ny;
          normal_interseccion[2] = nz;
        }
      }
    }

    // bases del cilindro (two circular planes at top and bottom)
    for (int s = -1; s <= 1; s += 2) {
      // normal to base (pointing outward)
      float nx = s * ux, ny = s * uy, nz = s * uz;
      // center of base
      float Px = Cx + 0.5f * h * nx, Py = Cy + 0.5f * h * ny, Pz = Cz + 0.5f * h * nz;

      // ray-plane intersection
      float denom = direccion[0] * nx + direccion[1] * ny + direccion[2] * nz;
      if (std::fabs(denom) < 1e-8f) {
        continue;  // parallel to plane
      }

      float lambda =
          ((Px - origen[0]) * nx + (Py - origen[1]) * ny + (Pz - origen[2]) * nz) / denom;
      if (lambda < 1e-3f || lambda >= dist_min) {
        continue;
      }

      // intersection point
      float Ix = origen[0] + direccion[0] * lambda;
      float Iy = origen[1] + direccion[1] * lambda;
      float Iz = origen[2] + direccion[2] * lambda;

      // check if within circular base radius
      float dx = Ix - Px, dy = Iy - Py, dz = Iz - Pz;
      if (dx * dx + dy * dy + dz * dz > r * r) {
        continue;
      }

      // check if hitting from outside
      float d        = direccion[0] * nx + direccion[1] * ny + direccion[2] * nz;
      sentido_afuera = (d < 0.0f);
      if (!sentido_afuera) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
      }

      dist_min               = lambda;
      objeto_tipo            = 1;
      objeto_idx             = i;
      punto_interseccion[0]  = Ix;
      punto_interseccion[1]  = Iy;
      punto_interseccion[2]  = Iz;
      normal_interseccion[0] = nx;
      normal_interseccion[1] = ny;
      normal_interseccion[2] = nz;
    }
  }

  // si no hay interseccion, retornar color de fondo (background gradient)
  if (objeto_tipo == -1) {
    // gradient based on Y direction: interpolate between light and dark
    float dir_y_norm = direccion[1];
    float m          = (dir_y_norm + 1.0f) / 2.0f;
    return {(1.0f - m) * bg_light[0] + m * bg_dark[0], (1.0f - m) * bg_light[1] + m * bg_dark[1],
            (1.0f - m) * bg_light[2] + m * bg_dark[2]};
  }

  // obtener material del objeto intersectado
  int mat_idx =
      (objeto_tipo == 0) ? esferas.material_idx[objeto_idx] : cilindros.material_idx[objeto_idx];

  // calcular reflexion segun tipo de material
  std::array<float, 3> dir_reflejada = {0.0f, 0.0f, 0.0f};
  std::array<float, 3> atenuacion    = {1.0f, 1.0f, 1.0f};
  calcular_reflexion(direccion, normal_interseccion, mat_idx, mats, mat_rng, sentido_afuera,
                     dir_reflejada, atenuacion);

  // calcular color recursivo (bounce the ray)
  std::array<float, 3> color_reflejado =
      calcular_color_rayo(punto_interseccion, dir_reflejada, profundidad - 1, esferas, cilindros,
                          mats, mat_rng, bg_dark, bg_light);

  // color atenuado (apply material reflectance)
  return {color_reflejado[0] * atenuacion[0], color_reflejado[1] * atenuacion[1],
          color_reflejado[2] * atenuacion[2]};
}

// trazado de rayos
// generates rays for each pixel and accumulates color samples
void generar_rayos_imagen(CameraConfig const & camera, RenderConfig const & render_config,
                          Spheres & esferas, Cylinders & cilindros, Materials & mats,
                          std::mt19937_64 & ray_rng, std::mt19937_64 & material_rng) {
  std::array<float, 3> origen_ventana, paso_x, paso_y;

  // initialize ray generation parameters (projection window)
  generacion_rayos(camera, render_config.image_width, render_config.image_height, origen_ventana,
                   paso_x, paso_y);

  // now origen_ventana, paso_x, and paso_y contain the calculated values
  for (int f = 0; f < render_config.image_height; f++) {   // loop through rows of image
    for (int c = 0; c < render_config.image_width; c++) {  // loop through columns of the image
      for (int i = 0; i < render_config.samples_per_pixel; i++) {  // making the rays for one pixel
        // generate random numbers in [-0.5, 0.5] for pixel jittering
        std::uniform_real_distribution<float> rand01(-0.5f, 0.5f);
        float delta_x = rand01(ray_rng);
        float delta_y = rand01(ray_rng);

        // Q = O + Δx*(c+δx) + Δy*(f+δy) (random position within pixel)
        std::array<float, 3> Q = {
          origen_ventana[0] + paso_x[0] * (c + delta_x) + paso_y[0] * (f + delta_y),
          origen_ventana[1] + paso_x[1] * (c + delta_x) + paso_y[1] * (f + delta_y),
          origen_ventana[2] + paso_x[2] * (c + delta_x) + paso_y[2] * (f + delta_y)};

        // rayo: origen = P (camera position), direccion = normalize(Q - P)
        std::array<float, 3> dir = {Q[0] - camera.position[0], Q[1] - camera.position[1],
                                    Q[2] - camera.position[2]};

        // calcular color del rayo con profundidad maxima
        std::array<float, 3> color = calcular_color_rayo(
            camera.position, normalize(dir), render_config.max_depth, esferas, cilindros, mats,
            material_rng, render_config.background_dark, render_config.background_light);
        // TODO: Store or accumulate color for averaging
      }
    }
  }
}

// funcion para calcular reflexion segun tipo de material
// calculates reflected/refracted ray direction and color attenuation based on material type
void calcular_reflexion(std::array<float, 3> const & direccion_original,
                        std::array<float, 3> const & normal, int material_idx, Materials & mats,
                        std::mt19937_64 & mat_rng, bool sentido_afuera,
                        std::array<float, 3> & dir_reflejada, std::array<float, 3> & atenuacion) {
  int tipo_material = mats.tipo[material_idx];  // 0=mate, 1=metal, 2=refractivo

  if (tipo_material == 0) {  // material mate (diffuse/Lambertian)
    // generar direccion aleatoria: normal + random vector
    std::uniform_real_distribution<float> rand_dist(-1.0f, 1.0f);
    float rand_x = rand_dist(mat_rng);
    float rand_y = rand_dist(mat_rng);
    float rand_z = rand_dist(mat_rng);

    // direccion = normal + random (diffuse scatter)
    dir_reflejada[0] = normal[0] + rand_x;
    dir_reflejada[1] = normal[1] + rand_y;
    dir_reflejada[2] = normal[2] + rand_z;

    // verificar si el vector es demasiado pequeno (degenerate case)
    if (std::fabs(dir_reflejada[0]) < 1e-8f &&
        std::fabs(dir_reflejada[1]) < 1e-8f &&
        std::fabs(dir_reflejada[2]) < 1e-8f)
    {
      dir_reflejada[0] = normal[0];
      dir_reflejada[1] = normal[1];
      dir_reflejada[2] = normal[2];
    }

    // atenuacion = reflectancia del material (color absorption)
    atenuacion[0] = mats.reflectancia_r[material_idx];
    atenuacion[1] = mats.reflectancia_g[material_idx];
    atenuacion[2] = mats.reflectancia_b[material_idx];

  } else if (tipo_material == 1) {  // material metalico (specular reflection with fuzz)
    // calcular d1r = do - 2(do·dn)dn (perfect mirror reflection)
    float dot_do_dn = direccion_original[0] * normal[0] +
                      direccion_original[1] * normal[1] +
                      direccion_original[2] * normal[2];

    std::array<float, 3> d1r = {direccion_original[0] - 2.0f * dot_do_dn * normal[0],
                                direccion_original[1] - 2.0f * dot_do_dn * normal[1],
                                direccion_original[2] - 2.0f * dot_do_dn * normal[2]};
    d1r                      = normalize(d1r);

    // generar vector de difusion (fuzz factor for imperfect reflection)
    float factor_difusion = mats.factor_difusion[material_idx];
    std::uniform_real_distribution<float> rand_dist(-factor_difusion, factor_difusion);
    float phi_x = rand_dist(mat_rng);
    float phi_y = rand_dist(mat_rng);
    float phi_z = rand_dist(mat_rng);

    // direccion reflejada = d1r normalizado + vector difusion
    dir_reflejada[0] = d1r[0] + phi_x;
    dir_reflejada[1] = d1r[1] + phi_y;
    dir_reflejada[2] = d1r[2] + phi_z;

    // atenuacion = reflectancia del material
    atenuacion[0] = mats.reflectancia_r[material_idx];
    atenuacion[1] = mats.reflectancia_g[material_idx];
    atenuacion[2] = mats.reflectancia_b[material_idx];

  } else if (tipo_material == 2) {  // material refractivo (glass, water, etc.)
    // normalizar direccion original
    std::array<float, 3> do_hat = normalize(direccion_original);

    // calcular angulo de refraccion usando Snell's law
    float dot_do_dn = -(do_hat[0] * normal[0] + do_hat[1] * normal[1] + do_hat[2] * normal[2]);
    float cos_theta = (dot_do_dn < 1.0f) ? dot_do_dn : 1.0f;
    float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);

    // indice de refraccion corregido (depends on if entering or exiting material)
    float rho       = mats.indice_refraccion[material_idx];
    float rho_prima = sentido_afuera ? rho : (1.0f / rho);

    // verificar caso de reflexion total interna
    if (rho_prima * sin_theta > 1.0f) {
      // reflexion total (total internal reflection, like in fiber optics)
      dir_reflejada[0] = do_hat[0] - 2.0f * dot_do_dn * normal[0];
      dir_reflejada[1] = do_hat[1] - 2.0f * dot_do_dn * normal[1];
      dir_reflejada[2] = do_hat[2] - 2.0f * dot_do_dn * normal[2];
    } else {
      // refraccion (Snell's law refraction)
      // u = ρ'(do + (cos θ)dn)
      std::array<float, 3> u = {rho_prima * (do_hat[0] + cos_theta * normal[0]),
                                rho_prima * (do_hat[1] + cos_theta * normal[1]),
                                rho_prima * (do_hat[2] + cos_theta * normal[2])};

      // v = -(√|1 - ||u||²|)dn
      float mag_u_sq         = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
      float raiz             = std::sqrt(std::fabs(1.0f - mag_u_sq));
      std::array<float, 3> v = {-raiz * normal[0], -raiz * normal[1], -raiz * normal[2]};

      // dr = u + v (refracted direction)
      dir_reflejada[0] = u[0] + v[0];
      dir_reflejada[1] = u[1] + v[1];
      dir_reflejada[2] = u[2] + v[2];
    }
    // atenuacion = 100% para materiales refractivos (no color absorption)
    atenuacion[0] = 1.0f;
    atenuacion[1] = 1.0f;
    atenuacion[2] = 1.0f;
  }
}
