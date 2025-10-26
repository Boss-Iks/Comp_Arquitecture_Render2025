#pragma once

#include "CommonTypes.hpp"
#include <cmath>

//un struct que contiene todos los parametros para generar la imagen
struct ConfigParams {
    AspectRatio aspect_ratio{16, 9};
    int image_width{1920};
    double gamma{2.2};

    Vec3 camera_position{0.0, 0.0, -10.0};
    Vec3 camera_target{0.0, 0.0, 0.0};
    Vec3 camera_north{0.0, 1.0, 0.0};
    double field_of_view{90.0};

    int samples_per_pixel{20};
    int max_depth{5};


    int material_rng_seed{13};
    int ray_rng_seed{19};


    Color background_dark_color{0.25, 0.5, 1.0};
    Color background_light_color{1.0, 1.0, 1.0};

 //calcula la altura de la imagen
    [[nodiscard]] int imageHeight() const {
        double ratio = static_cast<double>(aspect_ratio.height) / aspect_ratio.width;
        int height = static_cast<int>(std::round(static_cast<double>(image_width) * ratio));
        
        // Asegura que la altura sea al menos 1.
        return (height < 1) ? 1 : height;
    }
};