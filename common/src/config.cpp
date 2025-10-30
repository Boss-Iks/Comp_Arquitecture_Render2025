#include "../include/config.hpp"
#include <array>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>

namespace {

  inline void ltrim(std::string & s) {
    std::size_t i = 0;
    while (i < s.size() and std::isspace(static_cast<unsigned char>(s[i])) != 0) {
      ++i;
    }
    s.erase(0, i);
  }

  inline void rtrim(std::string & s) {
    if (s.empty()) {
      return;
    }
    std::size_t i = s.size();
    while (i > 0 and std::isspace(static_cast<unsigned char>(s[i - 1])) != 0) {
      --i;
    }
    s.erase(i);
  }

  inline void trim(std::string & s) {
    ltrim(s);
    rtrim(s);
  }

  [[noreturn]] void fail_unknown_key(std::string const & key) {
    std::cerr << "Error: Unknown configuration key: [" << key << "]\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_invalid_value(std::string const & key) {
    std::cerr << "Error: Invalid value for key: [" << key << "]\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_extra(std::string const & key, std::string const & tail) {
    std::cerr << "Error: Extra data after configuration value for key: [" << key
              << "] (Extra: " << tail << ")\n";
    std::exit(EXIT_FAILURE);
  }

  bool read_int(std::istringstream & iss, int & out) {
    long long v{};
    if (!(iss >> v)) {
      return false;
    }
    if (v < std::numeric_limits<int>::min() or v > std::numeric_limits<int>::max()) {
      return false;
    }
    out = static_cast<int>(v);
    return true;
  }

  bool read_double(std::istringstream & iss, double & out) {
    double v{};
    if (!(iss >> v)) {
      return false;
    }
    out = v;
    return true;
  }

  std::string remaining_tail(std::istringstream & iss) {
    std::string tail, chunk;
    std::ostringstream os;
    while (iss >> chunk) {
      if (!tail.empty()) {
        os << ' ';
      }
      os << chunk;
      tail = os.str();
    }
    return tail;
  }

  inline void handle_aspect_ratio(std::istringstream & iss, Config & cfg, std::string const & key) {
    int w{}, h{};
    if (!read_int(iss, w) or !read_int(iss, h) or w <= 0 or h <= 0) {
      fail_invalid_value(key);
    }
    std::string const tail = remaining_tail(iss);
    if (!tail.empty()) {
      fail_extra(key, tail);
    }
    cfg.aspect_w = w;
    cfg.aspect_h = h;
  }

  inline void handle_image_width(std::istringstream & iss, Config & cfg, std::string const & key) {
    int w{};
    if (!read_int(iss, w) or w <= 0) {
      fail_invalid_value(key);
    }
    std::string const tail = remaining_tail(iss);
    if (!tail.empty()) {
      fail_extra(key, tail);
    }
    cfg.image_width = w;
  }

  inline void handle_gamma(std::istringstream & iss, Config & cfg, std::string const & key) {
    double g{};
    if (!read_double(iss, g) or g <= 0.0) {
      fail_invalid_value(key);
    }
    std::string const tail = remaining_tail(iss);
    if (!tail.empty()) {
      fail_extra(key, tail);
    }
    cfg.gamma = g;
  }

  template <std::size_t N>
  inline bool read_n_doubles(std::istringstream & iss, std::array<double, N> & out, int n) {
    if (n < 0) {  // negative makes no sense
      return false;
    }

    std::size_t const wanted = N;
    auto const got           = static_cast<std::size_t>(n);
    if (got != wanted) {
      return false;
    }

    for (std::size_t i = 0; i < wanted; ++i) {
      double v{};
      if (!(iss >> v)) {
        return false;
      }
      out.at(i) = v;  // tidy-friendly, bounds-checked
    }
    return true;
  }

  inline void ensure_no_tail(std::istringstream & iss, std::string const & key) {
    std::string const tail = remaining_tail(iss);
    if (!tail.empty()) {
      fail_extra(key, tail);
    }
  }

  inline void handle_camera_position(std::istringstream & iss, Config & cfg,
                                     std::string const & key) {
    std::array<double, 3> v{};
    if (!read_n_doubles(iss, v, 3)) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    cfg.cam_pos = v;
  }

  inline void handle_camera_target(std::istringstream & iss, Config & cfg,
                                   std::string const & key) {
    std::array<double, 3> v{};
    if (!read_n_doubles(iss, v, 3)) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    cfg.cam_target = v;
  }

  inline void handle_camera_north(std::istringstream & iss, Config & cfg, std::string const & key) {
    std::array<double, 3> v{};
    if (!read_n_doubles(iss, v, 3)) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    cfg.cam_north = v;
  }

  inline void handle_field_of_view(std::istringstream & iss, Config & cfg,
                                   std::string const & key) {
    double f{};
    if (!read_double(iss, f) or f <= 0.0 or f >= 180.0) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    cfg.fov_deg = f;
  }

  inline void handle_samples_per_pixel(std::istringstream & iss, Config & cfg,
                                       std::string const & key) {
    int n{};
    if (!read_int(iss, n) or n <= 0) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    cfg.samples_per_pixel = n;
  }

  inline void handle_max_depth(std::istringstream & iss, Config & cfg, std::string const & key) {
    int n{};
    if (!read_int(iss, n) or n <= 0) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    cfg.max_depth = n;
  }

  inline void handle_seed(std::istringstream & iss, int & dst, std::string const & key) {
    int n{};
    if (!read_int(iss, n)) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    dst = n;
  }

  inline void handle_color(std::istringstream & iss, std::array<double, 3> & dst,
                           std::string const & key) {
    std::array<double, 3> v{};
    if (!read_n_doubles(iss, v, 3)) {
      fail_invalid_value(key);
    }
    if (v[0] < 0.0 or v[0] > 1.0 or v[1] < 0.0 or v[1] > 1.0 or v[2] < 0.0 or v[2] > 1.0) {
      fail_invalid_value(key);
    }
    ensure_no_tail(iss, key);
    dst = v;
  }

  void ensure_file_exists(std::filesystem::path const & p) {
    if (!std::filesystem::exists(p)) {
      std::cerr << "Error: Configuration file not found: " << p.string() << "\n";
      std::exit(EXIT_FAILURE);
    }
  }

  inline void add_aspect_and_general_handlers(
      std::unordered_map<std::string,
                         std::function<void(std::istringstream &, Config &, std::string const &)>> &
          handlers) {
    handlers.emplace("aspect_ratio",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_aspect_ratio(iss, cfg, key);
                     });

    handlers.emplace("image_width",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_image_width(iss, cfg, key);
                     });

    handlers.emplace("gamma", [](std::istringstream & iss, Config & cfg, std::string const & key) {
      handle_gamma(iss, cfg, key);
    });
  }

  inline void add_camera_handlers(
      std::unordered_map<std::string,
                         std::function<void(std::istringstream &, Config &, std::string const &)>> &
          handlers) {
    handlers.emplace("camera_position",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_camera_position(iss, cfg, key);
                     });

    handlers.emplace("camera_target",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_camera_target(iss, cfg, key);
                     });

    handlers.emplace("camera_north",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_camera_north(iss, cfg, key);
                     });

    handlers.emplace("field_of_view",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_field_of_view(iss, cfg, key);
                     });
  }

  inline void add_rendering_handlers(
      std::unordered_map<std::string,
                         std::function<void(std::istringstream &, Config &, std::string const &)>> &
          handlers) {
    handlers.emplace("samples_per_pixel",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_samples_per_pixel(iss, cfg, key);
                     });

    handlers.emplace("max_depth", [](std::istringstream & iss, Config & cfg,
                                     std::string const & key) { handle_max_depth(iss, cfg, key); });
  }

  inline void add_seed_and_bg_handlers(
      std::unordered_map<std::string,
                         std::function<void(std::istringstream &, Config &, std::string const &)>> &
          handlers) {
    handlers.emplace("material_rng_seed",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_seed(iss, cfg.material_rng_seed, key);
                     });

    handlers.emplace("ray_rng_seed",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_seed(iss, cfg.ray_rng_seed, key);
                     });

    handlers.emplace("background_dark_color",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_color(iss, cfg.bg_dark, key);
                     });

    handlers.emplace("background_light_color",
                     [](std::istringstream & iss, Config & cfg, std::string const & key) {
                       handle_color(iss, cfg.bg_light, key);
                     });
  }

  inline auto make_handlers() {
    using HandlerFn = std::function<void(std::istringstream &, Config &, std::string const &)>;
    std::unordered_map<std::string, HandlerFn> handlers;

    add_aspect_and_general_handlers(handlers);
    add_camera_handlers(handlers);
    add_rendering_handlers(handlers);
    add_seed_and_bg_handlers(handlers);

    return handlers;
  }

  inline auto const & get_handlers() {
    static auto const handlers = make_handlers();
    return handlers;
  }

  inline bool is_comment_or_empty_line(std::string const & line) {
    std::string tmp = line;
    trim(tmp);
    if (tmp.empty()) {
      return true;
    }
    return tmp[0] == '#';
  }

  inline void strip_trailing_colon(std::string & s) {
    while (!s.empty() and s.back() == ':') {
      s.pop_back();
    }
    trim(s);
  }

  inline bool split_key_and_rest(std::string const & line, std::string & key, std::string & rest) {
    std::string s = line;
    trim(s);

    if (auto pos = s.find(':'); pos != std::string::npos) {
      key  = s.substr(0, pos);
      rest = s.substr(pos + 1);
      trim(key);
      trim(rest);
      strip_trailing_colon(key);
      return true;
    }

    if (auto pos = s.find('='); pos != std::string::npos) {
      key  = s.substr(0, pos);
      rest = s.substr(pos + 1);
      trim(key);
      trim(rest);
      strip_trailing_colon(key);
      return true;
    }

    std::istringstream iss(s);
    if (!(iss >> key)) {
      return false;
    }
    std::getline(iss, rest);
    trim(rest);
    trim(key);
    strip_trailing_colon(key);
    return true;
  }

  inline void normalize_key(std::string & key) {
    trim(key);
    for (char & c : key) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (!key.empty() and key.back() == ':') {
      key.pop_back();
      trim(key);
    }
  }

  void process_config_line(std::string const & line, int /*line_no*/, Config & cfg) {
    if (is_comment_or_empty_line(line)) {
      return;
    }

    std::string key;
    std::string rest;
    if (!split_key_and_rest(line, key, rest)) {
      return;
    }

    normalize_key(key);
    auto const & handlers = get_handlers();
    auto it               = handlers.find(key);
    if (it == handlers.end()) {
      fail_unknown_key(key);
    }

    std::istringstream vss(rest);
    it->second(vss, cfg, key);
  }

}  // anonymous namespace

Config parse_config(std::string_view config_path) {
  std::filesystem::path const p{std::string(config_path)};
  ensure_file_exists(p);

  Config cfg;  // defaults
  std::ifstream fin(p);
  if (!fin) {
    std::cerr << "Error: Failed to open configuration file: " << p.string() << "\n";
    std::exit(EXIT_FAILURE);
  }

  std::string line;
  int line_no = 0;
  while (std::getline(fin, line)) {
    ++line_no;
    process_config_line(line, line_no, cfg);
  }

  return cfg;
}
