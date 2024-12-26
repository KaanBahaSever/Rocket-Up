#include "Environment.hpp"
#include "planets/Earth.hpp"
#include "planets/Planet.hpp"  // Add this include
#include <memory>
#include <cmath>

// Change from auto to proper type
std::unique_ptr<Planet> planet;

Environment::Environment(const Planets &_planet, const std::string &rail)
    : planetEnumn(_planet), rail(rail)
{
    switch (_planet)
    {
    case Planets::Mercury:
        break;
    case Planets::Venus:
        break;

    case Planets::Earth:
        planet = std::make_unique<Earth>();
        break;
    case Planets::Mars:
        break;
    case Planets::Jupiter:
        break;
    case Planets::Saturn:
        break;
    case Planets::Uranus:
        break;
    case Planets::Neptune:
        break;
    }
}

const auto &Environment::getPlanet() const
{
    return *planet;
}