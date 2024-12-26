#ifndef EARTH_HPP
#define EARTH_HPP

#include "Planet.hpp"
#include <string>
#include <cmath>
#include <array>

class Earth : public Planet
{
public:
    double getMass() const override;
    double getRadius() const override;
    double getGravity() const override;
    double getAtmosphereDensity() const override;
    std::string getName() const override;

    double pressure(double altitude) const override;
    double getTemperature(double altitude) const;

    void setSeaLevelTemperature(double temp);
    void setSeaLevelPressure(double pressure);

    // Calculate atmospheric properties at given altitude
    double temperature(double altitude) const;
    double density(double altitude) const;
    double speed_of_sound(double altitude);
    double dynamic_viscosity(double altitude);
    double kinematic_viscosity(double altitude);
    double get_random_wind_speed(double altitude);

private:
    static size_t findLayer(double altitude);
};

#endif // EARTH_HPP
