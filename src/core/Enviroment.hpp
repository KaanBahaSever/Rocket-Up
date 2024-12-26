#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <string>
#include "Planet.hpp"

class Environment {
public:
    Environment(const Planet& planet, const std::string& rail);

    const Planet& getPlanet() const;
    const std::string& getRail() const;

private:
    Planet planet;
    std::string rail;
};

#endif // ENVIRONMENT_HPP