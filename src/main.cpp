#include "planets/Earth.hpp"
#include "core/Environment.hpp"

#include <iostream>

int main() {
    auto earth = std::make_unique<Earth>();
    earth->setSeaLevelTemperature(15.0);
    Environment env(std::move(earth), "none");
    double pressure = env.getPlanet().getPressure(20000.0);
    
    std::cout << "Temperature at 20000m: " << earth->getTemperature(20000.0) << " K" << std::endl;
    std::cout << "Pressure at 20000m: " << pressure << " Pa" << std::endl;
    
    return 0;
}