#ifndef EARTH_HPP
#define EARTH_HPP

#include "Planet.hpp"
#include <string>
#include <cmath>
#include <array>

class Earth : public Planet
{
public:
    Earth();
    virtual ~Earth();

    double getMass() const override;
    double getRadius() const override;
    double getGravity() const override;
    double getAtmosphereDensity() const override;
    std::string getName() const override;

    double getPressure(double altitude) const override;
    double getTemperature(double altitude) const override;
    double getDensity(double altitude) const override;

    void setSeaLevelTemperature(double temp);
    void setSeaLevelPressure(double pressure);

    // Calculate atmospheric properties at given altitude
    double speedOfSound(double altitude);
    double dynmaicViscosity(double altitude);
    double kinematicViscosity(double altitude);
    double getRandomWindSpeed(double altitude);

private:
    static size_t findLayer(double altitude);
};

#endif // EARTH_HPP
