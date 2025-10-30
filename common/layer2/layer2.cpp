//segmentacion logica de las funciones que estan en la area logica 2
//funciones: creacion de rayos, interccion de rayos, contrbuicion de color de rayo  
#include <vector>
#include <cmath>
#include <random>
using namespace std;

float image_width  = 1920;
float image_height = 1080;
float aspect_ratio[2] = {16,9};
int samples_per_pixel = 20;
int max_depth         = 5;

float camera_position[3] = { 0.0f, 0.0f, -10.0f };
float camera_target[3]   = { 0.0f, 0.0f,   0.0f };
float camera_north[3]    = { 0.0f, 1.0f,   0.0f };
float fov_deg = 90.0f;//campo de vision

float background_dark_color[3]  = { 0.25f, 0.50f, 1.00f };
float background_light_color[3] = { 1.00f, 1.00f, 1.00f };

// these are hypothetical types
Materials materiales{};
Spheres   lista_esferas{};
Cylinders lista_cilindros{};

// PRNG states (Layer 1)
mt19937_64 ray_rng_state(19);
mt19937_64 material_rng_state(13);

// derived:  using the same names as in the document to not feel lost
float vf[3] = {//vector focal -> del punto de vista hasta el destino de la vision
    camera_position[0] - camera_target[0],
    camera_position[1] - camera_target[1],
    camera_position[2] - camera_target[2]
};
//distancia focal, magnitud del vector focal
float df = sqrt(vf[0]*vf[0] + vf[1]*vf[1] + vf[2]*vf[2]);
float pi = 3.1415927f;//for conversion
float alpha = (fov_deg*pi)/180;//campo de vision en radiants
float hp = 2.0f * tan(0.5f * alpha) * df;//altura ventana de proyeccion
float wp = hp * (aspect_ratio[0] / aspect_ratio[1]);//anchura ventana de proyeccion
//direction vectors
//cross product of camera and vf

float cross_vf_n_x=    vf[1]*camera_north[2]-vf[2]*camera_north[1];
float cross_vf_n_y=vf[2]*camera_north[0]-vf[0]*camera_north[2];
float cross_vf_n_z=vf[0]*camera_north[1]-vf[1]*camera_north[0];

//magnitude of the cross section
float mag_cross_vf_n = sqrt(cross_vf_n_x*cross_vf_n_x+cross_vf_n_y*cross_vf_n_y+cross_vf_n_z*cross_vf_n_z);

if (mag_cross_vf_n != 0){
    vector<float>u ={
        cross_vf_n_x/mag_cross_vf_n,
        cross_vf_n_y/mag_cross_vf_n,
        cross_vf_n_z/mag_cross_vf_n
    };
}
//cross product between vf and u
vector<float> v = {
    vf[1]*u[2]-vf[2]*u[1],
    vf[2]*u[0]-vf[0]*u[2],
    vf[0]*u[1]-vf[1]*u[0]
}

//vector horizontal y vertical
float ph[3] = { wp*u[0], wp*u[1], wp*[2] };
float pv[3] = { -hp*v[0], -hp*v[1], -hp*v[2] };
float paso_x[3] = { ph[0]/w, ph[1]/w, ph[2]/w };
float paso_y[3] = { pv[0]/h, pv[1]/h, pv[2]/h };
//O
float origen_ventana[3] = {
    camera_position[0] - vf[0] - 0.5f*(ph[0] + pv[0]) + 0.5f*(paso_x[0] + paso_y[0]),
    camera_position[1] - vf[1] - 0.5f*(ph[1] + pv[1]) + 0.5f*(paso_x[1] + paso_y[1]),
    camera_position[2] - vf[2] - 0.5f*(ph[2] + pv[2]) + 0.5f*(paso_x[2] + paso_y[2])
