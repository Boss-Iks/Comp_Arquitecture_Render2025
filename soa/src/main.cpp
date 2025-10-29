#include "../../common/include/cli.hpp"
#include "../../common/include/config.hpp"
#include <iostream>
#include <print>
#include <string_view>
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

  std::cout << "Config: " << cli.config_path << "\n";
  std::cout << "Scene:  " << cli.scene_path << "\n";
  std::cout << "Output: " << cli.output_path << "\n";
  std::cout << "CLI parsing OK \n";
  return 0;
}
