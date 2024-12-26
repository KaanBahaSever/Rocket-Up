#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <string>
#include "planets/planets.hpp"

class Environment {
public:
    Environment(const Planets& planet, const std::string& rail);

    const auto& getPlanet() const;
    const std::string& getRail() const;

private:
    Planets planetEnumn;
    std::string rail;
};

#endif