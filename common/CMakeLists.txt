add_library(common STATIC)

target_sources(common 
    PRIVATE 
        src/vector.cpp
        src/ppm_writer.cpp
        src/cli.cpp
        src/config.cpp
        src/camera.cpp
        src/scene.cpp
        
)

target_include_directories(common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(common PUBLIC Microsoft.GSL::GSL)
