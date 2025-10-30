#include "../include/scene.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>

namespace {

  // ---------- small utilities ----------
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

  inline bool is_comment_or_empty(std::string const & line) {
    std::string t = line;
    trim(t);
    return t.empty() or t[0] == '#';
  }

  [[noreturn]] void fail_not_found(std::filesystem::path const & p, char const * kind) {
    std::cerr << "Error: " << kind << " file not found: " << p.string() << "\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_duplicate_material(std::string const & name) {
    std::cerr << "Error: Material with name [" << name << "] already exists\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_invalid_material_params(std::string const & tag) {
    std::cerr << "Error: Invalid material parameters for: [" << tag << "]\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_extra_material_tail(std::string const & tag, std::string const & tail) {
    std::cerr << "Error: Extra data after material parameters for: [" << tag << "] (Extra: " << tail
              << ")\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_unknown_entity(std::string const & tag) {
    std::cerr << "Error: Unknown scene entity: " << tag << "\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_invalid_object_params(std::string const & tag) {
    std::cerr << "Error: Invalid object parameters for: [" << tag << "]\n";
    std::exit(EXIT_FAILURE);
  }

  [[noreturn]] void fail_unknown_material(std::string const & name) {
    std::cerr << "Error: Material not found: [" << name << "]\n";
    std::exit(EXIT_FAILURE);
  }

  inline std::string tail_tokens(std::istringstream & iss) {
    std::ostringstream os;
    std::string w;
    bool first = true;
    while (iss >> w) {
      if (!first) {
        os << ' ';
      }
      os << w;
      first = false;
    }
    return os.str();
  }

  inline bool read_double(std::istringstream & iss, double & out) {
    double v{};
    if (!(iss >> v)) {
      return false;
    }
    out = v;
    return true;
  }

  inline bool read_rgb(std::istringstream & iss, std::array<double, 3> & rgb) {
    for (int i = 0; i < 3; ++i) {
      if (!read_double(iss, rgb.at(static_cast<std::size_t>(i)))) {
        return false;
      }
    }
    // range check [0,1]
    return (rgb[0] >= 0.0 and rgb[0] <= 1.0) and
           (rgb[1] >= 0.0 and rgb[1] <= 1.0) and
           (rgb[2] >= 0.0 and rgb[2] <= 1.0);
  }

  inline bool read3(std::istringstream & iss, std::array<double, 3> & v) {
    for (int i = 0; i < 3; ++i) {
      double x{};
      if (!(iss >> x)) {
        return false;
      }
      v.at(static_cast<std::size_t>(i)) = x;
    }
    return true;
  }

  inline double length3(std::array<double, 3> const & v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  // MaterialTable definition so functions can refer to it.
  struct MaterialTable {
    std::unordered_map<std::string, std::uint32_t> name_to_id;
  };

  inline std::uint32_t mat_id_or_die(MaterialTable const & mt, std::string const & name) {
    auto it = mt.name_to_id.find(name);
    if (it == mt.name_to_id.end()) {
      fail_unknown_material(name);
    }
    return it->second;
  }

  // ---------- parsing of material lines ----------
  // MaterialTable is defined above.

  inline void add_matte(std::istringstream & iss, Scene & scn, MaterialTable & mt,
                        std::string const & tag) {
    std::string name;
    if (!(iss >> name)) {
      fail_invalid_material_params(tag);
    }

    if (mt.name_to_id.find(name) != mt.name_to_id.end()) {
      fail_duplicate_material(name);
    }

    std::array<double, 3> rgb{};
    if (!read_rgb(iss, rgb)) {
      fail_invalid_material_params(tag);
    }

    std::string extra = tail_tokens(iss);
    if (!extra.empty()) {
      fail_extra_material_tail(tag, extra);
    }

    Material m;
    m.name      = name;
    m.type      = MaterialType::Matte;
    m.matte.rgb = rgb;
    scn.materials.push_back(m);
    mt.name_to_id.emplace(name, static_cast<std::uint32_t>(scn.materials.size() - 1));
  }

  inline void add_metal(std::istringstream & iss, Scene & scn, MaterialTable & mt,
                        std::string const & tag) {
    std::string name;
    if (!(iss >> name)) {
      fail_invalid_material_params(tag);
    }

    if (mt.name_to_id.find(name) != mt.name_to_id.end()) {
      fail_duplicate_material(name);
    }

    std::array<double, 3> rgb{};
    if (!read_rgb(iss, rgb)) {
      fail_invalid_material_params(tag);
    }

    double diffusion{};
    if (!read_double(iss, diffusion) or diffusion < 0.0) {
      fail_invalid_material_params(tag);
    }

    std::string extra = tail_tokens(iss);
    if (!extra.empty()) {
      fail_extra_material_tail(tag, extra);
    }

    Material m;
    m.name            = name;
    m.type            = MaterialType::Metal;
    m.metal.rgb       = rgb;
    m.metal.diffusion = diffusion;
    scn.materials.push_back(m);
    mt.name_to_id.emplace(name, static_cast<std::uint32_t>(scn.materials.size() - 1));
  }

  inline void add_refractive(std::istringstream & iss, Scene & scn, MaterialTable & mt,
                             std::string const & tag) {
    std::string name;
    if (!(iss >> name)) {
      fail_invalid_material_params(tag);
    }

    if (mt.name_to_id.find(name) != mt.name_to_id.end()) {
      fail_duplicate_material(name);
    }

    double index{};
    if (!read_double(iss, index) or index <= 0.0) {
      fail_invalid_material_params(tag);
    }

    std::string extra = tail_tokens(iss);
    if (!extra.empty()) {
      fail_extra_material_tail(tag, extra);
    }

    Material m;
    m.name       = name;
    m.type       = MaterialType::Refractive;
    m.refr.index = index;
    scn.materials.push_back(m);
    mt.name_to_id.emplace(name, static_cast<std::uint32_t>(scn.materials.size() - 1));
  }

  inline void ensure_file_exists(std::filesystem::path const & p) {
    if (!std::filesystem::exists(p)) {
      fail_not_found(p, "Scene");
    }
  }

  inline void add_sphere(std::istringstream & iss, Scene & scn, MaterialTable const & mt,
                         std::string const & tag) {
    std::array<double, 3> c{};
    double r{};
    if (!read3(iss, c) or !(iss >> r) or r <= 0.0) {
      fail_invalid_object_params(tag);
    }

    std::string mname;
    if (!(iss >> mname)) {
      fail_invalid_object_params(tag);
    }

    std::string extra = tail_tokens(iss);
    if (!extra.empty()) {
      fail_extra_material_tail(tag,
                               extra);  // reuse message style: “Extra data after ... for: [tag]”
    }

    Sphere s{};
    s.center      = c;
    s.radius      = r;
    s.material_id = mat_id_or_die(mt, mname);
    scn.spheres.push_back(s);
  }

  inline void add_cylinder(std::istringstream & iss, Scene & scn, MaterialTable const & mt,
                           std::string const & tag) {
    std::array<double, 3> c{};
    double r{};
    std::array<double, 3> axis{};
    if (!read3(iss, c) or !(iss >> r) or r <= 0.0 or !read3(iss, axis)) {
      fail_invalid_object_params(tag);
    }

    if (length3(axis) == 0.0) {
      fail_invalid_object_params(tag);
    }

    std::string mname;
    if (!(iss >> mname)) {
      fail_invalid_object_params(tag);
    }

    std::string extra = tail_tokens(iss);
    if (!extra.empty()) {
      fail_extra_material_tail(tag, extra);
    }

    Cylinder cy{};
    cy.base_center = c;
    cy.radius      = r;
    cy.axis        = axis;  // we’ll normalize/interpret height later when building geometry
    cy.material_id = mat_id_or_die(mt, mname);
    scn.cylinders.push_back(cy);
  }

  // process one non-empty/non-comment scene line and update scene/material table
  inline void process_scene_line(std::string const & line, Scene & scn, MaterialTable & mt) {
    // Split "tag: rest" (or "tag rest" if no colon)
    std::string s = line;
    trim(s);
    std::string tag;
    std::string rest;
    if (auto pos = s.find(':'); pos != std::string::npos) {
      tag  = s.substr(0, pos);
      rest = s.substr(pos + 1);
    } else {
      std::istringstream iss(s);
      iss >> tag;
      std::getline(iss, rest);
    }
    trim(tag);
    trim(rest);

    // normalize tag (lowercase)
    for (char & c : tag) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    std::istringstream iss(rest);

    if (tag == "matte") {
      add_matte(iss, scn, mt, tag);
    } else if (tag == "metal") {
      add_metal(iss, scn, mt, tag);
    } else if (tag == "refractive") {
      add_refractive(iss, scn, mt, tag);
    } else if (tag == "sphere") {
      add_sphere(iss, scn, mt, tag);
    } else if (tag == "cylinder") {
      add_cylinder(iss, scn, mt, tag);
    } else {
      fail_unknown_entity(tag);
    }
  }

}  // end anonymous namespace

Scene parse_scene(std::string_view scene_path) {
  std::filesystem::path p{std::string(scene_path)};
  ensure_file_exists(p);

  Scene scn;
  MaterialTable mt;

  std::ifstream fin(p);
  if (!fin) {
    std::cerr << "Error: Failed to open scene file: " << p.string() << "\n";
    std::exit(EXIT_FAILURE);
  }

  std::string line;
  while (std::getline(fin, line)) {
    if (is_comment_or_empty(line)) {
      continue;
    }
    process_scene_line(line, scn, mt);
  }

  return scn;
}
