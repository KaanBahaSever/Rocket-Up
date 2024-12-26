#ifndef PLANET_HPP
#define PLANET_HPP

#include <string>
#include <planets/Earth.hpp>

class Planet {
public:
    // Constructors
    Planet(Earth earth);

    // Getters
    std::string getName() const;
    double getMass() const;
    double getRadius() const;

private:
    // Member variables
    std::string name;
    double mass;
    double radius;
};

#endif