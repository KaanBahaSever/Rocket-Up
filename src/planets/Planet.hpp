#ifndef PLANET_HPP
#define PLANET_HPP

#include <string>

class Planet {
public:
    // Virtual destructor for proper cleanup
    virtual ~Planet() = default;

    // Pure virtual methods defining planet properties
    virtual double getMass() const = 0;
    virtual double getRadius() const = 0;
    virtual double getGravity() const = 0;
    virtual double getAtmosphereDensity() const = 0;
    virtual std::string getName() const = 0;
};

#endif // PLANET_HPP