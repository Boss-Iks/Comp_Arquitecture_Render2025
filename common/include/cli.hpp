#pragma once
#include <string>
#include <string_view>
#include <vector>

struct CLIArgs {
  std::string config_path;
  std::string scene_path;
  std::string output_path;
};

// No C-style arrays in the interface; vector<string_view> is fine for clang-tidy.
CLIArgs parse_cli(std::vector<std::string_view> const & args, std::string_view exec_name);
