#include "../include/camera.hpp"
#include "../include/rayos.hpp"
#include "../include/scene.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace {

  constexpr double EPSILON = 1e-9;

  // helper para comparar arrays de doubles
  bool arrays_iguales(std::array<double, 3> const & a, std::array<double, 3> const & b,
                      double epsilon = EPSILON) {
    return std::abs(a[0] - b[0]) < epsilon and
           std::abs(a[1] - b[1]) < epsilon and
           std::abs(a[2] - b[2]) < epsilon;
  }

  // tests para estructura Pixel
  TEST(PixelTest, InicializacionCorrecta) {
    Pixel p{255, 128, 0};
    EXPECT_EQ(p.r, 255);
    EXPECT_EQ(p.g, 128);
    EXPECT_EQ(p.b, 0);
  }

  // tests para estructura Ray
  TEST(RayTest, InicializacionDefecto) {
    Ray rayo{};
    EXPECT_TRUE(arrays_iguales(rayo.origin, {0.0, 0.0, 0.0}));
    EXPECT_TRUE(arrays_iguales(rayo.direction, {0.0, 0.0, 0.0}));
  }

  TEST(RayTest, AsignacionValores) {
    Ray rayo{};
    rayo.origin    = {1.0, 2.0, 3.0};
    rayo.direction = {0.0, 1.0, 0.0};
    EXPECT_TRUE(arrays_iguales(rayo.origin, {1.0, 2.0, 3.0}));
    EXPECT_TRUE(arrays_iguales(rayo.direction, {0.0, 1.0, 0.0}));
  }

  // tests para estructura HitRecord
  TEST(HitRecordTest, InicializacionDefecto) {
    HitRecord hit{};
    EXPECT_FALSE(hit.hit);
    EXPECT_DOUBLE_EQ(hit.t, 1e10);
    EXPECT_EQ(hit.material_id, 0);
  }

  TEST(HitRecordTest, ActualizacionValores) {
    HitRecord hit{};
    hit.hit         = true;
    hit.t           = 5.5;
    hit.point       = {1.0, 2.0, 3.0};
    hit.normal      = {0.0, 1.0, 0.0};
    hit.material_id = 42;

    EXPECT_TRUE(hit.hit);
    EXPECT_DOUBLE_EQ(hit.t, 5.5);
    EXPECT_TRUE(arrays_iguales(hit.point, {1.0, 2.0, 3.0}));
    EXPECT_TRUE(arrays_iguales(hit.normal, {0.0, 1.0, 0.0}));
    EXPECT_EQ(hit.material_id, 42);
  }

  // tests para FramebufferSOA
  TEST(FramebufferSOATest, Redimensionamiento) {
    FramebufferSOA fb{};
    fb.R.resize(100);
    fb.G.resize(100);
    fb.B.resize(100);

    EXPECT_EQ(fb.R.size(), 100);
    EXPECT_EQ(fb.G.size(), 100);
    EXPECT_EQ(fb.B.size(), 100);
  }

  TEST(FramebufferSOATest, AsignacionValores) {
    FramebufferSOA fb{};
    fb.R.resize(3);
    fb.G.resize(3);
    fb.B.resize(3);

    fb.R[0] = 255;
    fb.G[1] = 128;
    fb.B[2] = 64;

    EXPECT_EQ(fb.R[0], 255);
    EXPECT_EQ(fb.G[1], 128);
    EXPECT_EQ(fb.B[2], 64);
  }

  // tests de integracion para escena simple
  class EscenaSimpleTest : public ::testing::Test {
  protected:
    void SetUp() override {
      // configurar camara simple
      camara.P            = {0.0, 0.0, 0.0};
      camara.D            = {0.0, 0.0, -1.0};
      camara.N            = {0.0, 1.0, 0.0};
      camara.image_width  = 100;
      camara.image_height = 100;
      camara.O            = {-1.0, 1.0, -1.0};
      camara.dx           = {0.02, 0.0, 0.0};
      camara.dy           = {0.0, -0.02, 0.0};

      // crear material simple
      Material mat{};
      mat.type      = MaterialType::Matte;
      mat.matte.rgb = {1.0, 0.0, 0.0};
      escena.materials.push_back(mat);

      // agregar esfera en frente de la camara
      Sphere esfera{};
      esfera.center      = {0.0, 0.0, -5.0};
      esfera.radius      = 1.0;
      esfera.material_id = 0;
      escena.spheres.push_back(esfera);
    }

    Camera camara{};
    Scene escena{};
  };

  TEST_F(EscenaSimpleTest, TraceRaysAOSNoLanza) {
    std::vector<Pixel> framebuffer;
    EXPECT_NO_THROW(trace_rays_aos(camara, escena, framebuffer));
    EXPECT_EQ(framebuffer.size(), 10'000);
  }

  TEST_F(EscenaSimpleTest, TraceRaysSOANoLanza) {
    FramebufferSOA framebuffer;
    EXPECT_NO_THROW(trace_rays_soa(camara, escena, framebuffer));
    EXPECT_EQ(framebuffer.R.size(), 10'000);
    EXPECT_EQ(framebuffer.G.size(), 10'000);
    EXPECT_EQ(framebuffer.B.size(), 10'000);
  }

  TEST_F(EscenaSimpleTest, AOSySOAProducenResultadosIguales) {
    std::vector<Pixel> fb_aos;
    FramebufferSOA fb_soa;

    trace_rays_aos(camara, escena, fb_aos);
    trace_rays_soa(camara, escena, fb_soa);

    ASSERT_EQ(fb_aos.size(), fb_soa.R.size());

    for (std::size_t i = 0; i < fb_aos.size(); ++i) {
      EXPECT_EQ(fb_aos[i].r, fb_soa.R[i]) << "Diferencia en indice " << i;
      EXPECT_EQ(fb_aos[i].g, fb_soa.G[i]) << "Diferencia en indice " << i;
      EXPECT_EQ(fb_aos[i].b, fb_soa.B[i]) << "Diferencia en indice " << i;
    }
  }

  // tests para escena con cilindro
  class EscenaCilindroTest : public ::testing::Test {
  protected:
    void SetUp() override {
      camara.P            = {0.0, 0.0, 0.0};
      camara.D            = {0.0, 0.0, -1.0};
      camara.N            = {0.0, 1.0, 0.0};
      camara.image_width  = 50;
      camara.image_height = 50;
      camara.O            = {-1.0, 1.0, -1.0};
      camara.dx           = {0.04, 0.0, 0.0};
      camara.dy           = {0.0, -0.04, 0.0};

      Material mat{};
      mat.type            = MaterialType::Metal;
      mat.metal.rgb       = {0.8, 0.8, 0.8};
      mat.metal.diffusion = 0.1;
      escena.materials.push_back(mat);

      Cylinder cilindro{};
      cilindro.base_center = {0.0, -1.0, -5.0};
      cilindro.axis        = {0.0, 2.0, 0.0};
      cilindro.radius      = 0.5;
      cilindro.material_id = 0;
      escena.cylinders.push_back(cilindro);
    }

    Camera camara{};
    Scene escena{};
  };

  TEST_F(EscenaCilindroTest, RenderizadoCilindro) {
    std::vector<Pixel> framebuffer;
    EXPECT_NO_THROW(trace_rays_aos(camara, escena, framebuffer));
    EXPECT_EQ(framebuffer.size(), 2'500);
  }

  // tests para escena vacia
  TEST(EscenaVaciaTest, SoloColorFondo) {
    Camera cam{};
    cam.P            = {0.0, 0.0, 0.0};
    cam.image_width  = 10;
    cam.image_height = 10;
    cam.O            = {-1.0, 1.0, -1.0};
    cam.dx           = {0.2, 0.0, 0.0};
    cam.dy           = {0.0, -0.2, 0.0};

    Scene escena_vacia{};
    std::vector<Pixel> framebuffer;

    trace_rays_aos(cam, escena_vacia, framebuffer);

    EXPECT_EQ(framebuffer.size(), 100);

    // verificar que todos los pixeles tienen color de fondo
    for (auto const & pixel : framebuffer) {
      EXPECT_GT(pixel.r, 0);
      EXPECT_GT(pixel.g, 0);
      EXPECT_GT(pixel.b, 0);
    }
  }

  // tests para multiples materiales
  TEST(MultipleMaterialesTest, TresMateriales) {
    Camera cam{};
    cam.P            = {0.0, 0.0, 0.0};
    cam.image_width  = 20;
    cam.image_height = 20;
    cam.O            = {-1.0, 1.0, -1.0};
    cam.dx           = {0.1, 0.0, 0.0};
    cam.dy           = {0.0, -0.1, 0.0};

    Scene escena{};

    // material mate
    Material mat1{};
    mat1.type      = MaterialType::Matte;
    mat1.matte.rgb = {1.0, 0.0, 0.0};
    escena.materials.push_back(mat1);

    // material metalico
    Material mat2{};
    mat2.type            = MaterialType::Metal;
    mat2.metal.rgb       = {0.0, 1.0, 0.0};
    mat2.metal.diffusion = 0.2;
    escena.materials.push_back(mat2);

    // material refractivo
    Material mat3{};
    mat3.type       = MaterialType::Refractive;
    mat3.refr.index = 1.5;
    escena.materials.push_back(mat3);

    // tres esferas con diferentes materiales
    Sphere s1{};
    s1.center      = {-2.0, 0.0, -5.0};
    s1.radius      = 0.5;
    s1.material_id = 0;
    escena.spheres.push_back(s1);

    Sphere s2{};
    s2.center      = {0.0, 0.0, -5.0};
    s2.radius      = 0.5;
    s2.material_id = 1;
    escena.spheres.push_back(s2);

    Sphere s3{};
    s3.center      = {2.0, 0.0, -5.0};
    s3.radius      = 0.5;
    s3.material_id = 2;
    escena.spheres.push_back(s3);

    std::vector<Pixel> framebuffer;
    EXPECT_NO_THROW(trace_rays_aos(cam, escena, framebuffer));
    EXPECT_EQ(framebuffer.size(), 400);
  }

  // tests de rendimiento basico
  TEST(RendimientoTest, ImagenGrande) {
    Camera cam{};
    cam.P            = {0.0, 0.0, 0.0};
    cam.image_width  = 200;
    cam.image_height = 200;
    cam.O            = {-1.0, 1.0, -1.0};
    cam.dx           = {0.01, 0.0, 0.0};
    cam.dy           = {0.0, -0.01, 0.0};

    Scene escena{};
    Material mat{};
    mat.type      = MaterialType::Matte;
    mat.matte.rgb = {0.5, 0.5, 0.5};
    escena.materials.push_back(mat);

    Sphere esfera{};
    esfera.center      = {0.0, 0.0, -5.0};
    esfera.radius      = 1.0;
    esfera.material_id = 0;
    escena.spheres.push_back(esfera);

    std::vector<Pixel> framebuffer;
    EXPECT_NO_THROW(trace_rays_aos(cam, escena, framebuffer));
    EXPECT_EQ(framebuffer.size(), 40'000);
  }

}  // namespace
