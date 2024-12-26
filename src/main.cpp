#include "planets/Earth.hpp"
#include "planets/Planets.hpp"
#include "core/Environment.hpp"

#include <iostream>

int main() {
    Earth earth;
    earth.setSeaLevelTemperature(15.0);
    Environment env(Planets::Earth, "none");
    double pressure = env.getPlanet().pressure(20000.0);
    
    std::cout << "Temperature at 20000m: " << earth.temperature(20000.0) << " K" << std::endl;
    std::cout << "Pressure at 20000m: " << pressure << " Pa" << std::endl;
    
    return 0;
}