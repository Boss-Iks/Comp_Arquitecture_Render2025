#pragma once

#include <iostream>


//una struct para almacenar los puntos de coordinados 3D (x,y,z)
struct Vec3 {
    double x{}, y{}, z{};
};

//overload para >> operador en un vec3
std::istream& operator>>(std::istream& is, Vec3& v);


//una estructura para almacenar niveles de colores
struct Color {
    double r{}, g{}, b{};
};

//overload para >> operador en un color
std::istream& operator>>(std::istream& is, Color& c);

//estructura para almacenar el aspect ratio de la imagen 
struct AspectRatio {
    int width{}, height{};
};

//overload para >> operador en el aspect ratio
std::istream& operator>>(std::istream& is, AspectRatio& ar);