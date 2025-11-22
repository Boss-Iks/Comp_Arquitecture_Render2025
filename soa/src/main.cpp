// include all the libraries
#include "camera.hpp"
#include "cli.hpp"
#include "color.hpp"
#include "config.hpp"
#include "ppm_writer.hpp"
#include "rayos.hpp"
#include "scene.hpp"
#include <cstdint>
#include <iostream>
#include <vector>
// #include "vector.hpp"

using namespace std;

int main(int argc, char * argv[]) {
  // std::println("Starting SOA rendering");
  // render::vector vec{1.0, 2.0, 3.0};
  // std::println("Vector magnitude: {}", vec.magnitude());

  std::vector<std::string_view> args;
  args.reserve(static_cast<size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);  // NOLINT
  }

  CLIArgs cli = parse_cli(args, "render-soa");
  Config cfg  = parse_config(cli.config_path);
  std::cout << "Config loaded (defaults): width=" << cfg.image_width << "\n";

  Scene scene = parse_scene(cli.scene_path);

  Camera cam = make_camera_from_config(cfg);
  std::cout << "Camera ready (" << cam.image_width << "x" << cam.image_height << ") \n";
  // Light sanity prints (avoid unused warnings)
  std::cout << "dx=(" << cam.dx[0] << "," << cam.dx[1] << "," << cam.dx[2] << ")\n";
  std::cout << "dy=(" << cam.dy[0] << "," << cam.dy[1] << "," << cam.dy[2] << ")\n";

  // Minimal output to avoid unused warnings and confirm flow
  std::cout << "Scene loaded (materials=" << scene.materials.size()
            << ", spheres=" << scene.spheres.size() << ", cylinders=" << scene.cylinders.size()
            << ") \n";

  std::cout << "Config: " << cli.config_path << "\n";
  std::cout << "Scene:  " << cli.scene_path << "\n";
  std::cout << "Output: " << cli.output_path << "\n";
  std::cout << "CLI parsing OK \n";
  return 0;
}
