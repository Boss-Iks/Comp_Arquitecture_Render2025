#include "../include/cli.hpp"
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <vector>

CLIArgs parse_cli(std::vector<std::string_view> const & args, std::string_view exec_name) {
  // args[0] is the executable name; we expect 4 tokens total
  if (args.size() != 4) {
    std::cerr << "Usage: " << exec_name << " <config.txt> <scene.txt> <output.ppm>\n";
    std::exit(EXIT_FAILURE);
  }

  CLIArgs out;
  out.config_path = std::string(args[1]);
  out.scene_path  = std::string(args[2]);
  out.output_path = std::string(args[3]);
  return out;
}
