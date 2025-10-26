#pragma once

#include <string>
#include <iosfwd>


struct ConfigParams;

//clase astetica para cargar parametros desde un stream o archivo
class ConfigLoader {
public:
    
    static ConfigParams loadFromStream(std::istream& stream);

    static ConfigParams loadFromFile(const std::string& filename);
};