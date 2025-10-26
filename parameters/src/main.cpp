#include "ConfigLoader.hpp"
#include "ConfigError.hpp"
#include "ConfigParams.hpp"

#include <iostream>
#include <format>

//imprimir los parametros de configuracion (ejemplo)
//cambia luego cuando llegamos a una otra fase de el proyecto
void printConfig(const ConfigParams& config) {
    std::cout << "--- Configuración Cargada ---\n";
    std::cout << std::format("  Aspect Ratio:    {}x{}\n", config.aspect_ratio.width, config.aspect_ratio.height);
    std::cout << std::format("  Image Width:     {}\n", config.image_width);
    std::cout << std::format("  Image Height:    {}\n", config.imageHeight()); // Muestra el valor calculado
    std::cout << std::format("  Gamma:           {}\n", config.gamma);
    std::cout << std::format("  Samples/Pixel:   {}\n", config.samples_per_pixel);
    std::cout << std::format("  Max Depth:       {}\n", config.max_depth);
    std::cout << std::format("  FOV:             {}\n", config.field_of_view);
    std::cout << std::format("  Cam Position:    ({}, {}, {})\n", config.camera_position.x, config.camera_position.y, config.camera_position.z);
    std::cout << std::format("  Cam Target:      ({}, {}, {})\n", config.camera_target.x, config.camera_target.y, config.camera_target.z);
    std::cout << std::format("  Cam North:       ({}, {}, {})\n", config.camera_north.x, config.camera_north.y, config.camera_north.z);
    std::cout << std::format("  Material Seed:   {}\n", config.material_rng_seed);
    std::cout << std::format("  Ray Seed:        {}\n", config.ray_rng_seed);
    std::cout << std::format("  BG Dark:         ({}, {}, {})\n", config.background_dark_color.r, config.background_dark_color.g, config.background_dark_color.b);
    std::cout << std::format("  BG Light:        ({}, {}, {})\n", config.background_light_color.r, config.background_light_color.g, config.background_light_color.b);
    std::cout << "-----------------------------\n";
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Uso: " << argv[0] << " <archivo_config.txt>\n";
        return 1;
    }

    std::string filename = argv[1];

    try {
        // Carga la configuración desde el archivo
        ConfigParams config = ConfigLoader::loadFromFile(filename);
        
        // Si tiene éxito, imprime los valores
        printConfig(config);
        
        // Aquí continuaría el resto del proyecto...
        //...
        //....
        //.......
        
    } catch (const ConfigError& e) {
        // Captura errores específicos de configuración
        std::cerr << e.what() << std::endl;
        return 1; 
    } catch (const std::exception& e) {
        // Captura otros errores inesperados
        std::cerr << "Error inesperado: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}