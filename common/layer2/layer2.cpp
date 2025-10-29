//segmentacion logica de las funciones que estan en la area logica 2
//funciones: creacion de rayos, interccion de rayos, contrbuicion de color de rayo  
#include "vector.hpp"

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
}
//given a w h , vactor<color>(w*h, {0,0,0})

void generate_image(config, scene, vista, imagen){
    
}
