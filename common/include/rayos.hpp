#ifndef RENDER_UTILS_HPP
#define RENDER_UTILS_HPP

#include <array>
#include <random>

// Data structure declarations (to be changed depending on how layer 1 calls it)
struct Spheres {
    int count;
    float* centro_x;
    float* centro_y;
    float* centro_z;
    float* radio;
    int* material_idx;
};

struct Cylinders {
    int count;
    float* centro_x;
    float* centro_y;
    float* centro_z;
    float* radio;
    float* eje_x;
    float* eje_y;
    float* eje_z;
    int* material_idx;
};

struct Materials {
    int* tipo; // 0=mate, 1=metal, 2=refractivo
    float* reflectancia_r;
    float* reflectancia_g;
    float* reflectancia_b;
    float* factor_difusion;
    float* indice_refraccion;
};

// Camera and render configuration struct (inputs the functions depend on (layer 1))
struct CameraConfig {
    std::array<float,3> position;
    std::array<float,3> target;
    std::array<float,3> north;
    float fov_deg;
    std::array<float,2> aspect_ratio;
};

struct RenderConfig {
    int image_width;
    int image_height;
    int samples_per_pixel;
    int max_depth;
    std::array<float,3> background_dark;
    std::array<float,3> background_light;
};

// Inline utility function (decrease memory call to fetch it on runtime)
inline std::array<float,3> normalize(const std::array<float,3>& a);

// Function declarations - now taking parameters instead of using globals
void generacion_rayos(
    const CameraConfig& camera,
    int image_width,
    int image_height,
    std::array<float,3>& origen_ventana,
    std::array<float,3>& paso_x,
    std::array<float,3>& paso_y
);

std::array<float,3> calcular_color_rayo(
    const std::array<float,3>& origen,
    const std::array<float,3>& direccion,
    int profundidad,
    Spheres& esferas,
    Cylinders& cilindros,
    Materials& mats,
    std::mt19937_64& mat_rng,
    const std::array<float,3>& bg_dark,
    const std::array<float,3>& bg_light
);

void generar_rayos_imagen(
    const CameraConfig& camera,
    const RenderConfig& render_config,
    Spheres& esferas,
    Cylinders& cilindros,
    Materials& mats,
    std::mt19937_64& ray_rng,
    std::mt19937_64& material_rng
);

void calcular_reflexion(
    const std::array<float,3>& direccion_original,
    const std::array<float,3>& normal,
    int material_idx,
    Materials& mats,
    std::mt19937_64& mat_rng,
    bool sentido_afuera,
    std::array<float,3>& dir_reflejada,
    std::array<float,3>& atenuacion
);

#endif
