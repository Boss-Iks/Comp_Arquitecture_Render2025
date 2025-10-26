cmake_minimum_required(VERSION 3.25)
project(ConfigLoaderProject CXX)

# --- Configuración del estándar C++23 ---
# Establece C++23 como el estándar para todo el proyecto
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS NO)

# --- Definir el ejecutable ---
# Agrega un ejecutable llamado 'config_loader_app'
# Incluimos los headers aquí para que aparezcan en los IDEs (como VSCode o CLion)
add_executable(config_loader_app
    src/main.cpp
    src/CommonTypes.cpp
    src/ConfigLoader.cpp
    src/CommonTypes.hpp
    src/ConfigError.hpp
    src/ConfigParams.hpp
    src/ConfigLoader.hpp
)

# --- Rutas de Inclusión ---
# Le dice al compilador que busque archivos #include "..."
# dentro del directorio 'src'.
target_include_directories(config_loader_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Nota: El uso de <format> (std::format) en C++23 está
# soportado nativamente por compiladores modernos (GCC 13+, Clang 17+, MSVC 19.34+).
# No se necesita linkear bibliotecas adicionales si tu compilador es reciente.
