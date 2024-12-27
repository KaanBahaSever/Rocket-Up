#include "Environment.hpp"
#include "planets/Earth.hpp"
#include "planets/Planet.hpp"
#include "planets/Planets.hpp"
#include <memory>
#include <cmath>

Environment::Environment(std::unique_ptr<Planet> planet, const std::string &rail)
    : planet(std::move(planet)), rail(rail) {}



