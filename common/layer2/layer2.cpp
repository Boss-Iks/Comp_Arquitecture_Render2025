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

vector<float> camera_position = { 0.0f, 0.0f, -10.0f };
vector<float> camera_target   = { 0.0f, 0.0f,   0.0f };
vector<float> camera_north    = { 0.0f, 1.0f,   0.0f };
float fov_deg = 90.0f;//campo de vision

vector<float> background_dark_color = { 0.25f, 0.50f, 1.00f };
vector<float>background_light_color = { 1.00f, 1.00f, 1.00f };

// these are hypothetical types
Materials materiales{};
Spheres   lista_esferas{};
Cylinders lista_cilindros{};

// PRNG states (Layer 1)
mt19937_64 ray_rng_state(19);
mt19937_64 material_rng_state(13);

//inline to tell the complier to just add it on the line of the code 
inline vector<float> normalize(const vector<float>& a) {
    float mag = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (mag > 1e-12f)
        return { a[0]/mag, a[1]/mag, a[2]/mag };
    else
        return { 0.0f, 0.0f, 0.0f };
}
// Function to initialize ray generation parameters and return them
void inicializar_generacion_rayos(vector<float>& origen_ventana, vector<float>& paso_x, vector<float>& paso_y) {
    
    // derived:  using the same names as in the document to not feel lost
    vector<float>vf = {//vector focal -> del punto de vista hasta el destino de la vision
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
    float cross_vf_n_x=vf[1]*camera_north[2]-vf[2]*camera_north[1];
    float cross_vf_n_y=vf[2]*camera_north[0]-vf[0]*camera_north[2];
    float cross_vf_n_z=vf[0]*camera_north[1]-vf[1]*camera_north[0];

    //magnitude of the cross section
    float mag_cross_vf_n = sqrt(cross_vf_n_x*cross_vf_n_x+cross_vf_n_y*cross_vf_n_y+cross_vf_n_z*cross_vf_n_z);

    vector<float> u ={
            (cross_vf_n_x/mag_cross_vf_n),
            (cross_vf_n_y/mag_cross_vf_n),
            (cross_vf_n_z/mag_cross_vf_n)
    };
    //cross product between vf and u
    vector<float> v = {
        vf[1]*u[2]-vf[2]*u[1],
        vf[2]*u[0]-vf[0]*u[2],
        vf[0]*u[1]-vf[1]*u[0]
    };

    //vector horizontal y vertical
    vector<float> ph = { 
        wp*u[0], 
        wp*u[1], 
        wp*u[2] };

    vector<float> pv = { 
        (-1.0f*hp)*v[0],
        (-1.0f*hp)*v[1],
        hp*(-1.0f*v[2]) };
    
    paso_x = { ph[0]/image_width, ph[1]/image_width, ph[2]/image_width };
    paso_y = { pv[0]/image_height, pv[1]/image_height, pv[2]/image_height };

    //O
    vector<float> half_psum= {
        0.5f*(ph[0] + pv[0]),
        0.5f*(ph[1] + pv[1]),
        0.5f*(ph[2] + pv[2]) 
    };

    vector<float> half_pasosum = {
        0.5f*(paso_x[0] + paso_y[0]),
        0.5f*(paso_x[1] + paso_y[1]),
        0.5f*(paso_x[2] + paso_y[2])    
    };

    origen_ventana = {
        camera_position[0] - vf[0] -half_psum[0]  +half_pasosum[0],
        camera_position[1] - vf[1] -half_psum[1] + half_pasosum[1],
        camera_position[2] - vf[2] - half_psum[2] + half_pasosum[2]
    };
}
// función para calcular color de un rayo
//returns the values that it calculates on updated> bg_dark/light and objects
vector<float> calcular_color_rayo( vector<float> origen,vector<float> direccion,int profundidad,Spheres& esferas,
    Cylinders& cilindros,Materials& mats,mt19937_64& mat_rng,vector<float>& bg_dark,vector<float>& bg_light){
    // si no hay profundidad, retornar negro
    if(profundidad <= 0){
        return {0.0f, 0.0f, 0.0f};
    }
    
    // buscar intersección más cercana
    float dist_min = 1e30f;
    int objeto_tipo = -1; // -1=ninguno, 0=esfera, 1=cilindro
    int objeto_idx = -1;
    vector<float> punto_interseccion = {0.0f, 0.0f, 0.0f};
    vector<float> normal_interseccion = {0.0f, 0.0f, 0.0f};
    bool sentido_afuera = true;
    
    // intersección con esferas
    for(int i = 0; i < esferas.count; i++){
        // rc = C - Or
        vector<float> rc = {
            esferas.centro_x[i] - origen[0],
            esferas.centro_y[i] - origen[1],
            esferas.centro_z[i] - origen[2]
        };
        
        // coeficientes ecuación cuadrática
        float a = direccion[0]*direccion[0] + direccion[1]*direccion[1] + direccion[2]*direccion[2];
        float b = 2.0f * (direccion[0]*rc[0] + direccion[1]*rc[1] + direccion[2]*rc[2]);
        float c = rc[0]*rc[0] + rc[1]*rc[1] + rc[2]*rc[2] - esferas.radio[i]*esferas.radio[i];
        
        float discriminante = b*b - 4.0f*a*c;
        
        if(discriminante >= 0.0f){
            float lambda1 = (-b - sqrt(discriminante))/(2.0f*a);
            float lambda2 = (-b + sqrt(discriminante))/(2.0f*a);
            
            float lambda = lambda1;
            if(lambda < 1e-3f) lambda = lambda2;
            
            if(lambda >= 1e-3f && lambda < dist_min){
                dist_min = lambda;
                objeto_tipo = 0;
                objeto_idx = i;
                
                // calcular punto e intersección
                punto_interseccion[0] = origen[0] + direccion[0]*lambda;
                punto_interseccion[1] = origen[1] + direccion[1]*lambda;
                punto_interseccion[2] = origen[2] + direccion[2]*lambda;
                
                // normal = (I - C)/r usando normalize
                vector<float> temp_normal = {
                    punto_interseccion[0] - esferas.centro_x[i],
                    punto_interseccion[1] - esferas.centro_y[i],
                    punto_interseccion[2] - esferas.centro_z[i]
                };
                normal_interseccion = normalize(temp_normal);
                
                // sentido
                float dot_dir_normal = direccion[0]*normal_interseccion[0] + 
                                      direccion[1]*normal_interseccion[1] + 
                                      direccion[2]*normal_interseccion[2];
                sentido_afuera = (dot_dir_normal < 0.0f);
                
                if(!sentido_afuera){
                    normal_interseccion[0] = -normal_interseccion[0];
                    normal_interseccion[1] = -normal_interseccion[1];
                    normal_interseccion[2] = -normal_interseccion[2];
                }
            }
        }
    }
    // interseccion con cilindros
    for (int i = 0; i < cilindros.count; i++) {
        // datos basicos
        float Cx = cilindros.centro_x[i], Cy = cilindros.centro_y[i], Cz = cilindros.centro_z[i];
        float r  = cilindros.radio[i];
        float ax = cilindros.eje_x[i], ay = cilindros.eje_y[i], az = cilindros.eje_z[i];

        // eje unitario y altura usando normalize
        vector<float> eje_vec = {ax, ay, az};
        float amag = sqrt(ax*ax + ay*ay + az*az); 
        if (amag < 1e-8f) continue;
        vector<float> u_eje = normalize(eje_vec);
        float ux = u_eje[0], uy = u_eje[1], uz = u_eje[2], h = amag;

        // rc = Or - C  (note: opposite sign than spheres)
        float rcx = origen[0]-Cx, rcy = origen[1]-Cy, rcz = origen[2]-Cz;

        // components perpendicular to axis assing all in one line to save space
        float ddot = direccion[0]*ux + direccion[1]*uy + direccion[2]*uz;
        float drx = direccion[0]-ddot*ux, dry = direccion[1]-ddot*uy, drz = direccion[2]-ddot*uz;

        float rdot = rcx*ux + rcy*uy + rcz*uz;
        float rcpx = rcx - rdot*ux, rcpy = rcy - rdot*uy, rcpz = rcz - rdot*uz;

        // quadratic for infinite cylinder
        float qa = drx*drx + dry*dry + drz*drz;
        float qb = 2.0f*(rcpx*drx + rcpy*dry + rcpz*drz);
        float qc = (rcpx*rcpx + rcpy*rcpy + rcpz*rcpz) - r*r;
        float disc = qb*qb - 4.0f*qa*qc;

        if (qa > 1e-12f && disc >= 0.0f) {
            float sdisc = sqrt(disc);
            float lambda = (-qb - sdisc)/(2.0f*qa);
            if (lambda < 1e-3f) lambda = (-qb + sdisc)/(2.0f*qa);

            if (lambda >= 1e-3f && lambda < dist_min) {
                // intersection point and height check
                float Ix = origen[0] + direccion[0]*lambda;
                float Iy = origen[1] + direccion[1]*lambda;
                float Iz = origen[2] + direccion[2]*lambda;

                float ICx = Ix - Cx, ICy = Iy - Cy, ICz = Iz - Cz;
                float proj = ICx*ux + ICy*uy + ICz*uz;
                if (fabsf(proj) <= 0.5f*h + 1e-6f) {
                    // lateral normal usando normalize
                    vector<float> temp_n = {ICx - proj*ux, ICy - proj*uy, ICz - proj*uz};
                    vector<float> n_norm = normalize(temp_n);
                    float nx = n_norm[0], ny = n_norm[1], nz = n_norm[2];
                    
                    float d = direccion[0]*nx + direccion[1]*ny + direccion[2]*nz;
                    sentido_afuera = (d < 0.0f); 
                    if (!sentido_afuera){ nx=-nx; ny=-ny; nz=-nz; }

                    dist_min = lambda; objeto_tipo = 1; objeto_idx = i;
                    punto_interseccion[0]=Ix; punto_interseccion[1]=Iy; punto_interseccion[2]=Iz;
                    normal_interseccion[0]=nx; normal_interseccion[1]=ny; normal_interseccion[2]=nz;
                }
            }
        }

        // bases of the cilinders (two circular planes)
        for (int s = -1; s <= 1; s += 2) {
            float nx = s*ux, ny = s*uy, nz = s*uz;
            float Px = Cx + 0.5f*h*nx, Py = Cy + 0.5f*h*ny, Pz = Cz + 0.5f*h*nz;

            float denom = direccion[0]*nx + direccion[1]*ny + direccion[2]*nz;
            if (fabsf(denom) < 1e-8f) continue;

            float lambda = ((Px-origen[0])*nx + (Py-origen[1])*ny + (Pz-origen[2])*nz) / denom;
            if (lambda < 1e-3f || lambda >= dist_min) continue;

            float Ix = origen[0] + direccion[0]*lambda;
            float Iy = origen[1] + direccion[1]*lambda;
            float Iz = origen[2] + direccion[2]*lambda;

            float dx = Ix-Px, dy = Iy-Py, dz = Iz-Pz;
            if (dx*dx + dy*dy + dz*dz > r*r) continue;

            float d = direccion[0]*nx + direccion[1]*ny + direccion[2]*nz;
            sentido_afuera = (d < 0.0f); 
            if (!sentido_afuera){ nx=-nx; ny=-ny; nz=-nz; }

            dist_min = lambda; objeto_tipo = 1; objeto_idx = i;
            punto_interseccion[0]=Ix; punto_interseccion[1]=Iy; punto_interseccion[2]=Iz;
            normal_interseccion[0]=nx; normal_interseccion[1]=ny; normal_interseccion[2]=nz;
        }
    }
    
    // si no hay intersección, retornar color de fondo
    if(objeto_tipo == -1){
        float dir_y_norm = direccion[1];
        float m = (dir_y_norm + 1.0f) / 2.0f;
        return {
            (1.0f - m)*bg_light[0] + m*bg_dark[0],
            (1.0f - m)*bg_light[1] + m*bg_dark[1],
            (1.0f - m)*bg_light[2] + m*bg_dark[2]
        };
    }
    // obtener material del objeto
    int mat_idx;

    if (objeto_tipo == 0) {
        mat_idx = esferas.material_idx[objeto_idx];
    } else {
        mat_idx = cilindros.material_idx[objeto_idx];
    }
    
    // calcular reflexión según tipo de material
    vector<float> dir_reflejada = {0.0f, 0.0f, 0.0f};
    vector<float> atenuacion = {1.0f, 1.0f, 1.0f};
    calcular_reflexion(direccion, normal_interseccion,mat_idx,mats,mat_rng,sentido_afuera,dir_reflejada,atenuacion);    
    // calcular color **recursivo**
    vector<float> color_reflejado = calcular_color_rayo(punto_interseccion,dir_reflejada,profundidad - 1,
        esferas,cilindros,mats,mat_rng,bg_dark,bg_light);
    //color attenuado
    vector<float> color_attenuado= {
        color_reflejado[0] * atenuacion[0],
        color_reflejado[1] * atenuacion[1],
        color_reflejado[2] * atenuacion[2]
    };
    return color_attenuado;
}
//trazado de rayos
void generar_rayos_imagen() {
    vector<float> origen_ventana, paso_x, paso_y;
    
    // Initialize ray generation parameters - these will be modified by reference
    inicializar_generacion_rayos(origen_ventana, paso_x, paso_y);
    
    // Now origen_ventana, paso_x, and paso_y contain the calculated values
    for ( int f = 0; f<image_height; f++){//loop through rows of image
        for (int c= 0 ; c<image_width;c++){//loop through colums of the image
            for (int i = 0; i <samples_per_pixel; i++){//making the rays for one pixel
                //generate random numbers
                uniform_real_distribution<float> rand01(-0.5f, 0.5f);
                float delta_x = rand01(ray_rng_state);
                float delta_y = rand01(ray_rng_state);
                // Q = O + Δx*(c+δx) + Δy*(f+δy)
                vector<float> Q = {
                    origen_ventana[0] + paso_x[0]*(c + delta_x) + paso_y[0]*(f + delta_y),
                    origen_ventana[1] + paso_x[1]*(c + delta_x) + paso_y[1]*(f + delta_y),
                    origen_ventana[2] + paso_x[2]*(c + delta_x) + paso_y[2]*(f + delta_y)
                };

                // rayo: origen = P, dirección = normalize(Q - P)
                vector<float> dir = {
                    Q[0] - camera_position[0],
                    Q[1] - camera_position[1],
                    Q[2] - camera_position[2]
                };
                            
                // calcular color del rayo con profundidad máxima
                vector<float> color = calcular_color_rayo(camera_position,normalize(dir),max_depth,lista_esferas,lista_cilindros,
                    materiales,material_rng_state,background_dark_color,background_light_color );
            }
        }
    }
}

    
// función para calcular reflexión según tipo de material
void calcular_reflexion(vector<float>& direccion_original,vector<float>& normal,int material_idx,Materials& mats,
    mt19937_64& mat_rng,bool sentido_afuera, vector<float>& dir_reflejada,vector<float>& atenuacion){
   
    int tipo_material = mats.tipo[material_idx]; // 0=mate, 1=metal, 2=refractivo    
    if(tipo_material == 0){ // material mate
        // generar dirección aleatoria
        uniform_real_distribution<float> rand_dist(-1.0f, 1.0f);
        float rand_x = rand_dist(mat_rng);
        float rand_y = rand_dist(mat_rng);
        float rand_z = rand_dist(mat_rng);
        
        // dirección = normal + random
        dir_reflejada[0] = normal[0] + rand_x;
        dir_reflejada[1] = normal[1] + rand_y;
        dir_reflejada[2] = normal[2] + rand_z;
        
        // verificar si el vector es demasiado pequeño
        if(fabsf(dir_reflejada[0]) < 1e-8f && 
           fabsf(dir_reflejada[1]) < 1e-8f && 
           fabsf(dir_reflejada[2]) < 1e-8f){
            dir_reflejada[0] = normal[0];
            dir_reflejada[1] = normal[1];
            dir_reflejada[2] = normal[2];
        }
        
        // atenuación = reflectancia del material
        atenuacion[0] = mats.reflectancia_r[material_idx];
        atenuacion[1] = mats.reflectancia_g[material_idx];
        atenuacion[2] = mats.reflectancia_b[material_idx];
    
    }else if(tipo_material == 1){ // material metálico
        // calcular d1r = do - 2(do·dn)dn
        float dot_do_dn = direccion_original[0]*normal[0] + 
                         direccion_original[1]*normal[1] + 
                         direccion_original[2]*normal[2];
        
        vector<float> d1r = {
            direccion_original[0] - 2.0f*dot_do_dn*normal[0],
            direccion_original[1] - 2.0f*dot_do_dn*normal[1],
            direccion_original[2] - 2.0f*dot_do_dn*normal[2]
        };
        
        // normalizar d1r usando helper function
        d1r = normalize(d1r);
        
        // generar vector de difusión
        float factor_difusion = mats.factor_difusion[material_idx];
        uniform_real_distribution<float> rand_dist(-factor_difusion, factor_difusion);
        float phi_x = rand_dist(mat_rng);
        float phi_y = rand_dist(mat_rng);
        float phi_z = rand_dist(mat_rng);
        
        // dirección reflejada = d1r normalizado + vector difusión
        dir_reflejada[0] = d1r[0] + phi_x;
        dir_reflejada[1] = d1r[1] + phi_y;
        dir_reflejada[2] = d1r[2] + phi_z;
        
        // atenuación = reflectancia del material
        atenuacion[0] = mats.reflectancia_r[material_idx];
        atenuacion[1] = mats.reflectancia_g[material_idx];
        atenuacion[2] = mats.reflectancia_b[material_idx];
    
    } else if(tipo_material == 2){ // material refractivo
        // normalizar dirección original usando helper function
        vector<float> do_hat = normalize(direccion_original);
        
        // calcular ángulo de refracción
        float dot_do_dn = -(do_hat[0]*normal[0] + do_hat[1]*normal[1] + do_hat[2]*normal[2]);
        float cos_theta = (dot_do_dn < 1.0f) ? dot_do_dn : 1.0f;
        float sin_theta = sqrt(1.0f - cos_theta*cos_theta);
        
        // índice de refracción corregido
        float rho = mats.indice_refraccion[material_idx];
        float rho_prima = sentido_afuera ? rho : (1.0f/rho);
        
        // verificar caso
        if(rho_prima * sin_theta > 1.0f){
            // reflexión total
            dir_reflejada[0] = do_hat[0] - 2.0f*dot_do_dn*normal[0];
            dir_reflejada[1] = do_hat[1] - 2.0f*dot_do_dn*normal[1];
            dir_reflejada[2] = do_hat[2] - 2.0f*dot_do_dn*normal[2];
        }
        else{
            // refracción
            // u = ρ(do + (cos θ)dn)
            vector<float> u = {
                rho_prima * (do_hat[0] + cos_theta*normal[0]),
                rho_prima * (do_hat[1] + cos_theta*normal[1]),
                rho_prima * (do_hat[2] + cos_theta*normal[2])
            };
            
            // v = -(√|1 - ||u||²|)dn
            //mag may end up being zero, to check on unit testing
            float mag_u_sq = u[0]*u[0] + u[1]*u[1] + u[2]*u[2];
            float raiz = sqrt(fabsf(1.0f - mag_u_sq));
            vector<float> v = {
                -raiz * normal[0],
                -raiz * normal[1],
                -raiz * normal[2]
            };
            
            // dr = u + v
            dir_reflejada[0] = u[0] + v[0];
            dir_reflejada[1] = u[1] + v[1];
            dir_reflejada[2] = u[2] + v[2];
        }
        
        // atenuación = 100% para materiales refractivos
        atenuacion[0] = 1.0f;
        atenuacion[1] = 1.0f;
        atenuacion[2] = 1.0f;
    }
}