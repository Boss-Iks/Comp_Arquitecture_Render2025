#include "rayos.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>
#include <random>
#include <vector>

// Constants to avoid magic numbers
namespace TestConstants {

  constexpr float kZero           = 0.0F;
  constexpr float kOne            = 1.0F;
  constexpr float kMinusOne       = -1.0F;
  constexpr float kMinusFive      = -5.0F;
  constexpr float kThree          = 3.0F;
  constexpr float kFour           = 4.0F;
  constexpr float kTwo            = 2.0F;
  constexpr float kHalf           = 0.5F;
  constexpr float kPointSix       = 0.6F;
  constexpr float kPointEight     = 0.8F;
  constexpr float kPointThree     = 0.3F;
  constexpr float kPointNine      = 0.9F;
  constexpr float kPointSeven     = 0.7F;
  constexpr float kOnePointFive   = 1.5F;
  constexpr float kPointOne       = 0.1F;
  constexpr float kNinety         = 90.0F;
  constexpr float kFortyFive      = 45.0F;
  constexpr float kSixteen        = 16.0F;
  constexpr float kNine           = 9.0F;
  constexpr float kTolerance      = 1e-6F;
  constexpr float kSmallTolerance = 0.01F;
  constexpr float kMagTolerance   = 0.1F;

  constexpr int kImageWidth       = 800;
  constexpr int kImageHeight      = 450;
  constexpr int kImageWidthLarge  = 1'600;
  constexpr int kImageHeightLarge = 900;
  constexpr int kSamplesPerPixel  = 10;
  constexpr int kMaxDepth         = 5;
  constexpr int kZeroDepth        = 0;
  constexpr int kSingleObject     = 1;

  constexpr int kRngSeed1    = 123;
  constexpr int kRngSeed2    = 456;
  constexpr int kRngSeedMain = 12'345;

  constexpr int kMatTypeBase       = 0;
  constexpr int kMatTypeMetal      = 1;
  constexpr int kMatTypeRefractive = 2;

}  // namespace TestConstants

// Test fixture class for ray tracing tests
class RayTracingTest : public ::testing::Test {
protected:
  void SetUp() override {
    InitializeCamera();
    InitializeRenderConfig();
    InitializeSpheres();
    InitializeCylinders();
    InitializeMaterials();
    InitializeRng();
  }

  void TearDown() override {
    CleanupSpheres();
    CleanupCylinders();
    CleanupMaterials();
  }

  void InitializeCamera() {
    using namespace TestConstants;
    camera.position     = {kZero, kZero, kZero};
    camera.target       = {kZero, kZero, kMinusOne};
    camera.north        = {kZero, kOne, kZero};
    camera.fov_deg      = kNinety;
    camera.aspect_ratio = {kSixteen, kNine};
  }

  void InitializeRenderConfig() {
    using namespace TestConstants;
    render_config.image_width       = kImageWidth;
    render_config.image_height      = kImageHeight;
    render_config.samples_per_pixel = kSamplesPerPixel;
    render_config.max_depth         = kMaxDepth;
    render_config.background_dark   = {kHalf, kPointSeven, kOne};
    render_config.background_light  = {kOne, kOne, kOne};
  }

  void InitializeSpheres() {
    using namespace TestConstants;
    sphere_data_x        = std::vector<float>{kZero};
    sphere_data_y        = std::vector<float>{kZero};
    sphere_data_z        = std::vector<float>{kMinusFive};
    sphere_data_radio    = std::vector<float>{kOne};
    sphere_data_material = std::vector<int>{kMatTypeBase};

    spheres.count        = kSingleObject;
    spheres.centro_x     = sphere_data_x.data();
    spheres.centro_y     = sphere_data_y.data();
    spheres.centro_z     = sphere_data_z.data();
    spheres.radio        = sphere_data_radio.data();
    spheres.material_idx = sphere_data_material.data();
  }

  void InitializeCylinders() {
    using namespace TestConstants;
    cylinder_data_x        = std::vector<float>{kThree};
    cylinder_data_y        = std::vector<float>{kZero};
    cylinder_data_z        = std::vector<float>{kMinusFive};
    cylinder_data_radio    = std::vector<float>{kHalf};
    cylinder_data_eje_x    = std::vector<float>{kZero};
    cylinder_data_eje_y    = std::vector<float>{kTwo};
    cylinder_data_eje_z    = std::vector<float>{kZero};
    cylinder_data_material = std::vector<int>{kMatTypeMetal};

    cylinders.count        = kSingleObject;
    cylinders.centro_x     = cylinder_data_x.data();
    cylinders.centro_y     = cylinder_data_y.data();
    cylinders.centro_z     = cylinder_data_z.data();
    cylinders.radio        = cylinder_data_radio.data();
    cylinders.eje_x        = cylinder_data_eje_x.data();
    cylinders.eje_y        = cylinder_data_eje_y.data();
    cylinders.eje_z        = cylinder_data_eje_z.data();
    cylinders.material_idx = cylinder_data_material.data();
  }

  void InitializeMaterials() {
    using namespace TestConstants;
    mat_data_tipo              = std::vector<int>{kMatTypeBase, kMatTypeMetal, kMatTypeRefractive};
    mat_data_reflectancia_r    = std::vector<float>{kPointEight, kPointNine, kOne};
    mat_data_reflectancia_g    = std::vector<float>{kPointThree, kPointNine, kOne};
    mat_data_reflectancia_b    = std::vector<float>{kPointThree, kPointNine, kOne};
    mat_data_factor_difusion   = std::vector<float>{kZero, kPointOne, kZero};
    mat_data_indice_refraccion = std::vector<float>{kOne, kOne, kOnePointFive};

    materials.tipo              = mat_data_tipo.data();
    materials.reflectancia_r    = mat_data_reflectancia_r.data();
    materials.reflectancia_g    = mat_data_reflectancia_g.data();
    materials.reflectancia_b    = mat_data_reflectancia_b.data();
    materials.factor_difusion   = mat_data_factor_difusion.data();
    materials.indice_refraccion = mat_data_indice_refraccion.data();
  }

  void InitializeRng() { rng.seed(TestConstants::kRngSeedMain); }

  void CleanupSpheres() {
    // Vectors handle their own cleanup
  }

  void CleanupCylinders() {
    // Vectors handle their own cleanup
  }

  void CleanupMaterials() {
    // Vectors handle their own cleanup
  }

  CameraConfig camera;
  RenderConfig render_config;
  Spheres spheres;
  Cylinders cylinders;
  Materials materials;
  std::mt19937_64 rng;

  // Use vectors instead of raw pointers for RAII
  std::vector<float> sphere_data_x;
  std::vector<float> sphere_data_y;
  std::vector<float> sphere_data_z;
  std::vector<float> sphere_data_radio;
  std::vector<int> sphere_data_material;

  std::vector<float> cylinder_data_x;
  std::vector<float> cylinder_data_y;
  std::vector<float> cylinder_data_z;
  std::vector<float> cylinder_data_radio;
  std::vector<float> cylinder_data_eje_x;
  std::vector<float> cylinder_data_eje_y;
  std::vector<float> cylinder_data_eje_z;
  std::vector<int> cylinder_data_material;

  std::vector<int> mat_data_tipo;
  std::vector<float> mat_data_reflectancia_r;
  std::vector<float> mat_data_reflectancia_g;
  std::vector<float> mat_data_reflectancia_b;
  std::vector<float> mat_data_factor_difusion;
  std::vector<float> mat_data_indice_refraccion;
};

// ============================================================
// Tests for normalize() function
// ============================================================

TEST(NormalizeTest, NormalizesUnitVector) {
  using namespace TestConstants;
  std::array<float, 3> vector = {kOne, kZero, kZero};
  std::array<float, 3> result = normalize(vector);
  EXPECT_FLOAT_EQ(result[0], kOne);
  EXPECT_FLOAT_EQ(result[1], kZero);
  EXPECT_FLOAT_EQ(result[2], kZero);
}

TEST(NormalizeTest, NormalizesNonUnitVector) {
  using namespace TestConstants;
  std::array<float, 3> vector = {kThree, kFour, kZero};
  std::array<float, 3> result = normalize(vector);
  EXPECT_FLOAT_EQ(result[0], kPointSix);
  EXPECT_FLOAT_EQ(result[1], kPointEight);
  EXPECT_FLOAT_EQ(result[2], kZero);
}

TEST(NormalizeTest, HandlesZeroVector) {
  using namespace TestConstants;
  std::array<float, 3> vector = {kZero, kZero, kZero};
  std::array<float, 3> result = normalize(vector);
  EXPECT_FLOAT_EQ(result[0], kZero);
  EXPECT_FLOAT_EQ(result[1], kZero);
  EXPECT_FLOAT_EQ(result[2], kZero);
}

TEST(NormalizeTest, NormalizesNegativeVector) {
  using namespace TestConstants;
  std::array<float, 3> vector = {kMinusOne, kMinusOne, kMinusOne};
  std::array<float, 3> result = normalize(vector);
  float expected              = kMinusOne / std::numbers::sqrt3_v<float>;
  EXPECT_NEAR(result[0], expected, kTolerance);
  EXPECT_NEAR(result[1], expected, kTolerance);
  EXPECT_NEAR(result[2], expected, kTolerance);
}

// ============================================================
// Tests for generacion_rayos() function
// ============================================================

TEST_F(RayTracingTest, GeneracionRayosOutputsValidVectors) {
  using namespace TestConstants;
  std::array<float, 3> origen_ventana{};
  std::array<float, 3> paso_x{};
  std::array<float, 3> paso_y{};

  generacion_rayos(camera, render_config.image_width, render_config.image_height, origen_ventana,
                   paso_x, paso_y);

  EXPECT_NE(origen_ventana[0], kZero);
  EXPECT_NE(paso_x[0], kZero);
  EXPECT_NE(paso_y[0], kZero);
}

TEST_F(RayTracingTest, GeneracionRayosStepSizeProportionalToImageSize) {
  using namespace TestConstants;
  std::array<float, 3> origen_ventana1{};
  std::array<float, 3> paso_x1{};
  std::array<float, 3> paso_y1{};
  std::array<float, 3> origen_ventana2{};
  std::array<float, 3> paso_x2{};
  std::array<float, 3> paso_y2{};

  generacion_rayos(camera, kImageWidth, kImageHeight, origen_ventana1, paso_x1, paso_y1);
  generacion_rayos(camera, kImageWidthLarge, kImageHeightLarge, origen_ventana2, paso_x2, paso_y2);

  float mag_paso_x1 =
      std::sqrt(paso_x1[0] * paso_x1[0] + paso_x1[1] * paso_x1[1] + paso_x1[2] * paso_x1[2]);
  float mag_paso_x2 =
      std::sqrt(paso_x2[0] * paso_x2[0] + paso_x2[1] * paso_x2[1] + paso_x2[2] * paso_x2[2]);

  EXPECT_NEAR(mag_paso_x1 / mag_paso_x2, kTwo, kSmallTolerance);
}

TEST_F(RayTracingTest, GeneracionRayosFOVAffectsStepSize) {
  using namespace TestConstants;
  std::array<float, 3> origen_ventana1{};
  std::array<float, 3> paso_x1{};
  std::array<float, 3> paso_y1{};
  std::array<float, 3> origen_ventana2{};
  std::array<float, 3> paso_x2{};
  std::array<float, 3> paso_y2{};

  CameraConfig camera2 = camera;
  camera2.fov_deg      = kFortyFive;

  generacion_rayos(camera, kImageWidth, kImageHeight, origen_ventana1, paso_x1, paso_y1);
  generacion_rayos(camera2, kImageWidth, kImageHeight, origen_ventana2, paso_x2, paso_y2);

  float mag_paso_x1 =
      std::sqrt(paso_x1[0] * paso_x1[0] + paso_x1[1] * paso_x1[1] + paso_x1[2] * paso_x1[2]);
  float mag_paso_x2 =
      std::sqrt(paso_x2[0] * paso_x2[0] + paso_x2[1] * paso_x2[1] + paso_x2[2] * paso_x2[2]);

  EXPECT_LT(mag_paso_x2, mag_paso_x1);
}

// ============================================================
// Tests for calcular_color_rayo() - Background
// ============================================================

TEST_F(RayTracingTest, ColorRayoReturnsBackgroundWhenNoIntersection) {
  using namespace TestConstants;
  std::array<float, 3> origen    = {kZero, kZero, kZero};
  std::array<float, 3> direccion = {kZero, kZero, kOne};

  Spheres empty_spheres     = {0, nullptr, nullptr, nullptr, nullptr, nullptr};
  Cylinders empty_cylinders = {0,       nullptr, nullptr, nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr};

  std::array<float, 3> color =
      calcular_color_rayo(origen, direccion, kMaxDepth, empty_spheres, empty_cylinders, materials,
                          rng, render_config.background_dark, render_config.background_light);

  EXPECT_GT(color[0] + color[1] + color[2], kZero);
}

TEST_F(RayTracingTest, ColorRayoReturnsBlackAtZeroDepth) {
  using namespace TestConstants;
  std::array<float, 3> origen    = {kZero, kZero, kZero};
  std::array<float, 3> direccion = {kZero, kZero, kMinusOne};

  std::array<float, 3> color =
      calcular_color_rayo(origen, direccion, kZeroDepth, spheres, cylinders, materials, rng,
                          render_config.background_dark, render_config.background_light);

  EXPECT_FLOAT_EQ(color[0], kZero);
  EXPECT_FLOAT_EQ(color[1], kZero);
  EXPECT_FLOAT_EQ(color[2], kZero);
}

// ============================================================
// Tests for calcular_color_rayo() - Sphere Intersection
// ============================================================

TEST_F(RayTracingTest, ColorRayoDetectsSphereIntersection) {
  using namespace TestConstants;
  std::array<float, 3> origen    = {kZero, kZero, kZero};
  std::array<float, 3> direccion = {kZero, kZero, kMinusOne};

  Cylinders empty_cylinders = {0,       nullptr, nullptr, nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr};

  std::array<float, 3> color =
      calcular_color_rayo(origen, direccion, kMaxDepth, spheres, empty_cylinders, materials, rng,
                          render_config.background_dark, render_config.background_light);

  EXPECT_GT(color[0] + color[1] + color[2], kZero);
}

TEST_F(RayTracingTest, ColorRayoIgnoresSphereBehindOrigin) {
  using namespace TestConstants;
  std::array<float, 3> origen    = {kZero, kZero, kZero};
  std::array<float, 3> direccion = {kZero, kZero, kOne};

  Cylinders empty_cylinders = {0,       nullptr, nullptr, nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr};

  std::array<float, 3> color =
      calcular_color_rayo(origen, direccion, kMaxDepth, spheres, empty_cylinders, materials, rng,
                          render_config.background_dark, render_config.background_light);

  EXPECT_GT(color[0], kHalf);
}

// ============================================================
// Tests for calcular_reflexion() - Diffuse Material
// ============================================================

TEST_F(RayTracingTest, ReflexionDiffuseMaterialScattersRandomly) {
  using namespace TestConstants;
  std::array<float, 3> direccion = {kZero, kZero, kMinusOne};
  std::array<float, 3> normal    = {kZero, kZero, kOne};
  std::array<float, 3> dir_reflejada1{};
  std::array<float, 3> dir_reflejada2{};
  std::array<float, 3> atenuacion1{};
  std::array<float, 3> atenuacion2{};

  std::mt19937_64 rng1(kRngSeed1);
  std::mt19937_64 rng2(kRngSeed2);

  calcular_reflexion(direccion, normal, kMatTypeBase, materials, rng1, true, dir_reflejada1,
                     atenuacion1);
  calcular_reflexion(direccion, normal, kMatTypeBase, materials, rng2, true, dir_reflejada2,
                     atenuacion2);

  bool is_different = (dir_reflejada1[0] != dir_reflejada2[0]) or
                      (dir_reflejada1[1] != dir_reflejada2[1]) or
                      (dir_reflejada1[2] != dir_reflejada2[2]);
  EXPECT_TRUE(is_different);
}

TEST_F(RayTracingTest, ReflexionDiffuseAppliesCorrectAttenuation) {
  using namespace TestConstants;
  std::array<float, 3> direccion = {kZero, kZero, kMinusOne};
  std::array<float, 3> normal    = {kZero, kZero, kOne};
  std::array<float, 3> dir_reflejada{};
  std::array<float, 3> atenuacion{};

  calcular_reflexion(direccion, normal, kMatTypeBase, materials, rng, true, dir_reflejada,
                     atenuacion);

  EXPECT_FLOAT_EQ(atenuacion[0], mat_data_reflectancia_r[kMatTypeBase]);
  EXPECT_FLOAT_EQ(atenuacion[1], mat_data_reflectancia_g[kMatTypeBase]);
  EXPECT_FLOAT_EQ(atenuacion[2], mat_data_reflectancia_b[kMatTypeBase]);
}

// ============================================================
// Tests for calcular_reflexion() - Metal Material
// ============================================================

TEST_F(RayTracingTest, ReflexionMetalReflectsCorrectly) {
  using namespace TestConstants;
  std::array<float, 3> direccion = normalize(std::array<float, 3>{kZero, kMinusOne, kMinusOne});
  std::array<float, 3> normal    = {kZero, kZero, kOne};
  std::array<float, 3> dir_reflejada{};
  std::array<float, 3> atenuacion{};

  calcular_reflexion(direccion, normal, kMatTypeMetal, materials, rng, true, dir_reflejada,
                     atenuacion);

  EXPECT_GT(dir_reflejada[2], kZero);
}

// ============================================================
// Tests for calcular_reflexion() - Refractive Material
// ============================================================

TEST_F(RayTracingTest, ReflexionRefractiveMaterialNoAttenuation) {
  using namespace TestConstants;
  std::array<float, 3> direccion = {kZero, kZero, kMinusOne};
  std::array<float, 3> normal    = {kZero, kZero, kOne};
  std::array<float, 3> dir_reflejada{};
  std::array<float, 3> atenuacion{};

  calcular_reflexion(direccion, normal, kMatTypeRefractive, materials, rng, true, dir_reflejada,
                     atenuacion);

  EXPECT_FLOAT_EQ(atenuacion[0], kOne);
  EXPECT_FLOAT_EQ(atenuacion[1], kOne);
  EXPECT_FLOAT_EQ(atenuacion[2], kOne);
}

TEST_F(RayTracingTest, ReflexionRefractiveHandlesTotalInternalReflection) {
  using namespace TestConstants;
  std::array<float, 3> direccion = normalize(std::array<float, 3>{kOne, kZero, kOne});
  std::array<float, 3> normal    = {kZero, kZero, kOne};
  std::array<float, 3> dir_reflejada{};
  std::array<float, 3> atenuacion{};

  calcular_reflexion(direccion, normal, kMatTypeRefractive, materials, rng, false, dir_reflejada,
                     atenuacion);

  float magnitude = std::sqrt(dir_reflejada[0] * dir_reflejada[0] +
                              dir_reflejada[1] * dir_reflejada[1] +
                              dir_reflejada[2] * dir_reflejada[2]);
  EXPECT_GT(magnitude, kMagTolerance);
}

// ============================================================
// Main function to run all tests
// ============================================================

auto main(int argc, char ** argv) -> int {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
