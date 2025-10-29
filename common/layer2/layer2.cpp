//segmentacion logica de las funciones que estan en la area logica 2
//funciones: creacion de rayos, interccion de rayos, contrbuicion de color de rayo  
#include <vector>
#include <cmath>
#include <random>
using namespace std;
struct soa{
    vector<unsigned char> R,G,B;//values 0-255 2^8 (1B needed)
    //contructor 
    soa(int w, int h):
          R(w * h, 0),
          G(w * h, 0),
          B(w * h, 0)             
    {}
        
};

//initialization of aos
struct color {
    unsigned char R;
    unsigned char G;
    unsigned char B;
};
//given a w h , vactor<color>(w*h, {0,0,0})

//these are the variables needed for creating a point of view (then rays and whatnot) initialised with their default values (to be subed with actual values later, config file)
vector<float> camera_position = {0.0f, 0.0f, -10.0f};
vector<float> camera_target   = {0.0f, 0.0f,  0.0f};
vector<float> camera_north    = {0.0f, 1.0f,  0.0f};

float field_of_view = 90.0f;

int aspect_ratio_w = 16;
int aspect_ratio_h = 9;

int image_width  = 1920;
int image_height = image_width * aspect_ratio_h / aspect_ratio_w;  // 1080

unsigned int ray_rng_seed = 19;



struct vista {
  float origen_ventana, paso_x, paso_y,
  eje_u, eje_v, dir_view,
  rng_rayos,rand_x,rand_y;
};

vista punto_de_vista( const vector<float>& camera_position,const vector<float>& camera_target,const vector<float>& camera_north,
    float field_of_view,int aspect_ratio_w,int aspect_ratio_h,int image_width,int image_height,unsigned int ray_rng_seed){
   
    //init the variables
    vector<float> vector_focal(3);
    vector<float> dir_view(3);
    vector<float> eje_u(3);
    vector<float> eje_v(3);
    vector<float> proy_h(3);
    vector<float> proy_v(3);
    vector<float> paso_x(3);
    vector<float> paso_y(3);
    vector<float> origen_ventana(3);

   // focal vector (target - position)
    for (int i = 0; i < 3; ++i)
        vector_focal[i] = (camera_target[i] - camera_position[i]);

    // focal distance and normalized view direction
    float distancia_focal = std::sqrt(
        (vector_focal[0] * vector_focal[0]) +
        (vector_focal[1] * vector_focal[1]) +
        (vector_focal[2] * vector_focal[2])
    );

    if (distancia_focal > 0.0f) {
        for (int i = 0; i < 3; ++i)
            dir_view[i] = (vector_focal[i] / distancia_focal);
    } else {
        dir_view = {0.0f, 0.0f, 1.0f};
    }

    // right axis = normalize(north × view)
    eje_u[0] = ((camera_north[1] * dir_view[2]) - (camera_north[2] * dir_view[1]));
    eje_u[1] = ((camera_north[2] * dir_view[0]) - (camera_north[0] * dir_view[2]));
    eje_u[2] = ((camera_north[0] * dir_view[1]) - (camera_north[1] * dir_view[0]));

    //  (|u|) eje_u = sqrt(x² + y² + z²)
    float len_u = std::sqrt(
        (eje_u[0] * eje_u[0]) +
        (eje_u[1] * eje_u[1]) +
        (eje_u[2] * eje_u[2])
    );

    if (len_u > 0.0f) { // normalize eje_u -> make it a unit vector
        float inv_len_u = (1.0f / len_u);
        for (int i = 0; i < 3; ++i)
            eje_u[i] = (eje_u[i] * inv_len_u);
    }

    // up axis = view × right
    eje_v[0] = ((dir_view[1] * eje_u[2]) - (dir_view[2] * eje_u[1]));
    eje_v[1] = ((dir_view[2] * eje_u[0]) - (dir_view[0] * eje_u[2]));
    eje_v[2] = ((dir_view[0] * eje_u[1]) - (dir_view[1] * eje_u[0]));

    // window dimensions (world units)
    const float deg2rad = (3.14159265358979323846f / 180.0f);
    float half_fov      = (0.5f * field_of_view * deg2rad);
    float ventana_altura  = (2.0f * tan(half_fov) * distancia_focal);
    float ventana_anchura = (ventana_altura * (aspect_ratio_w /aspect_ratio_h));

    // project window edges
    for (int i = 0; i < 3; ++i) {
        proy_h[i] = (eje_u[i] * ventana_anchura);
        proy_v[i] = (-eje_v[i] * ventana_altura);
    }

    // pixel steps
    float inv_w = (1.0f / image_width);
    float inv_h = (1.0f / image_height);
    for (int i = 0; i < 3; ++i) {
        paso_x[i] = (proy_h[i] * inv_w);
        paso_y[i] = (proy_v[i] * inv_h);
    }

    // window origin
    for (int i = 0; i < 3; ++i) {
        float back    = (vector_focal[i]);
        float half_h  = (0.5f * (proy_h[i] + proy_v[i]));
        float half_px = (0.5f * (paso_x[i] + paso_y[i]));
        origen_ventana[i] = ((camera_position[i]) - (back) - (half_h) + (half_px));
    }

    //ray gen
    // initialize RNG (Mersenne Twister 64-bit) with the given seed
    mt19937_64 rng_rayos(ray_rng_seed);

    // uniform real distribution between -0.5 and +0.5
    uniform_real_distribution<float> rand_x(-0.5f, 0.5f);
    uniform_real_distribution<float> rand_y(-0.5f, 0.5f);

    vista output_view;
        output_view.origen_ventana;
         output_view.paso_x;
          output_view.paso_y;
          output_view.eje_u;
           output_view.eje_v;
            output_view.dir_view;
            output_view.rng_rayos;
             output_view.rand_x;
             output_view.rand_y;
    return output_view ;
};