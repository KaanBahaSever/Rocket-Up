#ifndef EARTH_HPP
#define EARTH_HPP

#include <cmath>
#include <array>

class Earth
{
public:
    static void setSeaLevelTemperature(double tempCelsius);
    static void setSeaLevelPressure(double pressure);

    // Calculate atmospheric properties at given altitude
    static double temperature(double altitude);
    static double pressure(double altitude);
    static double density(double altitude);
    static double speed_of_sound(double altitude);
    static double dynamic_viscosity(double altitude);
    static double kinematic_viscosity(double altitude);
    static double get_random_wind_speed(double altitude);

private:
    static size_t findLayer(double altitude);
};

#endif // EARTH_HPP
