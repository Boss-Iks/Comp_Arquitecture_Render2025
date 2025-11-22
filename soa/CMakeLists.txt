add_executable(render-soa)
target_sources(render-soa 
    PRIVATE 
      src/main.cpp
)
target_include_directories(render-soa PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/common/include  )

target_link_libraries(render-soa PRIVATE Microsoft.GSL::GSL common)