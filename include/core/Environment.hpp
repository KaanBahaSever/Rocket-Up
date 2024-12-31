#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include "../planets/Planet.hpp"
#include <memory>
#include <string>

class Environment
{
private:
    std::unique_ptr<Planet> planet;
    std::string rail;

public:
    Environment(std::unique_ptr<Planet> planet, const std::string &rail);
    const Planet &getPlanet() const { return *planet; }
};

#endif