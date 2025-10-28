//this is a sample just so layer 3 can push foward without waiting for previous layers
struct data_example {
    int width;
    int height;
    float gamma;
    float colour[3][6];  // [canal][píxel]
};

// Dummy que simula la salida de Layer 2 + gamma de Layer 1
data_example aos() {
    data_example aos;
    aos.width  = 3;    // ancho (número de columnas)
    aos.height = 4;    // alto (número de filas)
    aos.gamma  = 2.2f; // gamma leído en la parte 1
//f is to force the values to be float (smaller than double and required by the project to force float)
    // Colores promedio (ya en [0,1]) de Layer 2
    // canal 0 = R, 1 = G, 2 = B; 6 píxeles de ejemplo
    aos.colour[0][0] = 0.25f; aos.colour[1][0] = 0.50f; aos.colour[2][0] = 1.00f;
    aos.colour[0][1] = 0.82f; aos.colour[1][1] = 0.77f; aos.colour[2][1] = 0.65f;
    aos.colour[0][2] = 0.10f; aos.colour[1][2] = 0.10f; aos.colour[2][2] = 0.10f;
    aos.colour[0][3] = 0.95f; aos.colour[1][3] = 0.00f; aos.colour[2][3] = 0.00f;
    aos.colour[0][4] = 0.85f; aos.colour[1][4] = 0.85f; aos.colour[2][4] = 0.25f;
    aos.colour[0][5] = 0.05f; aos.colour[1][5] = 0.60f; aos.colour[2][5] = 0.10f;

    return aos;
}

struct data_example_soa {
    int width;
    int height;
    float gamma;
    float R[6];  // canal rojo
    float G[6];  // canal verde
    float B[6];  // canal azul
};

// Dummy que simula la salida de Layer 2 (SOA) + gamma de Layer 1
data_example_soa soa() {
    data_example_soa soa;
    soa.width  = 3;
    soa.height = 4;
    soa.gamma  = 2.2f;  // de parte 1

    // Colores promedio por píxel (rango [0,1]) — 6 píxeles de ejemplo
    soa.R[0] = 0.25f; soa.G[0] = 0.50f; soa.B[0] = 1.00f;
    soa.R[1] = 0.82f; soa.G[1] = 0.77f; soa.B[1] = 0.65f;
    soa.R[2] = 0.10f; soa.G[2] = 0.10f; soa.B[2] = 0.10f;
    soa.R[3] = 0.95f; soa.G[3] = 0.00f; soa.B[3] = 0.00f;
    soa.R[4] = 0.85f; soa.G[4] = 0.85f; soa.B[4] = 0.25f;
    soa.R[5] = 0.05f; soa.G[5] = 0.60f; soa.B[5] = 0.10f;

    return soa;
}