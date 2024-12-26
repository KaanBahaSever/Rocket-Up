#include "planets/Earth.hpp"

#include <iostream>

int main() {
    Earth::setSeaLevelTemperature(15.0);

    std::cout << "Temperature at 20000m: " << Earth::temperature(20000.0) << " K" << std::endl;
    
    return 0;
}