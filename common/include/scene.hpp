#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// ---------- Materials ----------
enum class MaterialType { Matte, Metal, Refractive };

struct Matte {
  std::array<double, 3> rgb{};  // [0,1]
};

struct Metal {
  std::array<double, 3> rgb{};  // [0,1]
  double diffusion{};           // >= 0 in validation
};

struct Refractive {
  double index{};  // > 0 in validation
};

struct Material {
  std::string name;  // unique
  MaterialType type{};
  // Discriminated union by 'type' (we avoid std::variant now to keep this bite minimal)
  Matte matte{};
  Metal metal{};
  Refractive refr{};
};

// ---------- Objects ----------
enum class ObjectType { Sphere, Cylinder };

struct Sphere {
  std::array<double, 3> center{};
  double radius{};              // > 0
  std::uint32_t material_id{};  // index into materials
};

struct Cylinder {
  std::array<double, 3> base_center{};
  double radius{};               // > 0
  std::array<double, 3> axis{};  // direction vector, length=height (validated later)
  std::uint32_t material_id{};   // index into materials
};

// ---------- Scene ----------
struct Scene {
  std::vector<Material> materials;
  std::vector<Sphere> spheres;
  std::vector<Cylinder> cylinders;
};

// Step 3.A: only verifies the file exists and returns an empty Scene.
// Parsing will be implemented in Step 3.B.
Scene parse_scene(std::string_view scene_path);
