#include "ConfigLoader.hpp"
#include "ConfigParams.hpp"
#include "ConfigError.hpp"
#include "CommonTypes.hpp" 

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <functional>
#include <stdexcept>
#include <format> 

// Inicio del Espacio Anónimo
namespace {

// Alias para el tipo de función de parseo
using ParserFunc = std::function<void(std::stringstream&, ConfigParams&, const std::string&)>;

//lanza una excepcion por un valor invalido
[[noreturn]] void throwInvalidValue(const std::string& key, const std::string& line) {
    throw ConfigError(std::format("Error: Invalid value for key: [{}]\nLine: \"{}\"", key, line));
}

//comprueba se queda datos extra en la linea
void checkForExtraData(std::stringstream& ss, const std::string& key, const std::string& line) {
    std::string extra_token;
    if (ss >> extra_token) {
        // Si leemos un token extra, recogemos el resto de la línea
        std::string remaining_line;
        std::getline(ss, remaining_line); // Obtiene el resto
        std::string all_extra = extra_token + remaining_line;
        
        throw ConfigError(std::format(
            "Error: Extra data after configuration value for key: [{}]\nExtra: \"{}\"", 
            key, all_extra));
    }
}

//  Funciones de Parseo Individuales 

void parseAspectRatio(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    AspectRatio ar;
    if (!(ss >> ar) || ar.width <= 0 || ar.height <= 0) {
        throwInvalidValue("aspect_ratio:", line);
    }
    checkForExtraData(ss, "aspect_ratio:", line);
    params.aspect_ratio = ar;
}

void parseImageWidth(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    int width;
    if (!(ss >> width) || width <= 0) {
        throwInvalidValue("image_width:", line);
    }
    checkForExtraData(ss, "image_width:", line);
    params.image_width = width;
}

void parseGamma(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    double gamma;
    if (!(ss >> gamma)) {
        throwInvalidValue("gamma:", line);
    }
    checkForExtraData(ss, "gamma:", line);
    params.gamma = gamma;
}

void parseCameraPosition(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    Vec3 pos;
    if (!(ss >> pos)) {
        throwInvalidValue("camera_position:", line);
    }
    checkForExtraData(ss, "camera_position:", line);
    params.camera_position = pos;
}

void parseCameraTarget(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    Vec3 target;
    if (!(ss >> target)) {
        throwInvalidValue("camera_target:", line);
    }
    checkForExtraData(ss, "camera_target:", line);
    params.camera_target = target;
}

void parseCameraNorth(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    Vec3 north;
    if (!(ss >> north)) {
        throwInvalidValue("camera_north:", line);
    }
    checkForExtraData(ss, "camera_north:", line);
    params.camera_north = north;
}

void parseFieldOfView(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    double fov;
    if (!(ss >> fov) || fov <= 0.0 || fov >= 180.0) {
        throwInvalidValue("field_of_view:", line);
    }
    checkForExtraData(ss, "field_of_view:", line);
    params.field_of_view = fov;
}

void parseSamplesPerPixel(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    int samples;
    if (!(ss >> samples) || samples <= 0) {
        throwInvalidValue("samples_per_pixel:", line);
    }
    checkForExtraData(ss, "samples_per_pixel:", line);
    params.samples_per_pixel = samples;
}

void parseMaxDepth(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    int depth;
    if (!(ss >> depth) || depth <= 0) {
        throwInvalidValue("max_depth:", line);
    }
    checkForExtraData(ss, "max_depth:", line);
    params.max_depth = depth;
}

void parseMaterialRngSeed(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    int seed;
    if (!(ss >> seed) || seed <= 0) {
        throwInvalidValue("material_rng_seed:", line);
    }
    checkForExtraData(ss, "material_rng_seed:", line);
    params.material_rng_seed = seed;
}

void parseRayRngSeed(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    int seed;
    if (!(ss >> seed) || seed <= 0) {
        throwInvalidValue("ray_rng_seed:", line);
    }
    checkForExtraData(ss, "ray_rng_seed:", line);
    params.ray_rng_seed = seed;
}

void parseBackgroundDarkColor(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    Color color;
    if (!(ss >> color) || color.r < 0.0 || color.r > 1.0 ||
                          color.g < 0.0 || color.g > 1.0 ||
                          color.b < 0.0 || color.b > 1.0) {
        throwInvalidValue("background_dark_color:", line);
    }
    checkForExtraData(ss, "background_dark_color:", line);
    params.background_dark_color = color;
}

void parseBackgroundLightColor(std::stringstream& ss, ConfigParams& params, const std::string& line) {
    Color color;
    if (!(ss >> color) || color.r < 0.0 || color.r > 1.0 ||
                          color.g < 0.0 || color.g > 1.0 ||
                          color.b < 0.0 || color.b > 1.0) {
        throwInvalidValue("background_light_color:", line);
    }
    checkForExtraData(ss, "background_light_color:", line);
    params.background_light_color = color;
}



const std::map<std::string, ParserFunc> parsers = {
    {"aspect_ratio:",          parseAspectRatio},
    {"image_width:",           parseImageWidth},
    {"gamma:",                 parseGamma},
    {"camera_position:",       parseCameraPosition},
    {"camera_target:",         parseCameraTarget},
    {"camera_north:",          parseCameraNorth},
    {"field_of_view:",         parseFieldOfView},
    {"samples_per_pixel:",     parseSamplesPerPixel},
    {"max_depth:",             parseMaxDepth},
    {"material_rng_seed:",     parseMaterialRngSeed},
    {"ray_rng_seed:",          parseRayRngSeed},
    {"background_dark_color:", parseBackgroundDarkColor},
    {"background_light_color:",parseBackgroundLightColor}
};

} // Fin del Espacio Anónimo 


// Implementación de Métodos Públicos de ConfigLoader 

ConfigParams ConfigLoader::loadFromStream(std::istream& stream) {
    ConfigParams params; // Comienza con los valores por defecto
    std::string line;
    
    while (std::getline(stream, line)) {
        std::stringstream ss(line);
        std::string key;
        
        ss >> key; // Extrae la primera palabra (la etiqueta)
        
        if (key.empty()) {
            continue; 
        }
        
        auto it = parsers.find(key);
        if (it == parsers.end()) {
            // Etiqueta no reconocida
            throw ConfigError(std::format("Error: Unknown configuration key: [{}]", key));
        }
        
        // Llama a la función de parseo correspondiente
        // La función de parseo manejará los errores de valor y datos extra
        it->second(ss, params, line);
    }
    
    return params;
}

ConfigParams ConfigLoader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw ConfigError(std::format("Error: Could not open configuration file: {}", filename));
    }
    return loadFromStream(file);
}