
#pragma once
#include <cmath>
#include <array>
//#include "Vector3.hpp"

class Atmosphere {
private:
    static constexpr double SCALE_HEIGHT = 7400.0;  // meters
    static constexpr double SEA_LEVEL_DENSITY = 1.225;  // kg/m^3
    static constexpr double GRAVITATIONAL_ACCELERATION = 9.80665;  // m/s^2

public:
    static double getDensity(double altitude) {
        return SEA_LEVEL_DENSITY * std::exp(-altitude / SCALE_HEIGHT);
    }

    static double getPressure(double altitude) {
        return SEA_LEVEL_DENSITY * GRAVITATIONAL_ACCELERATION * SCALE_HEIGHT * 
               std::exp(-altitude / SCALE_HEIGHT);
    }

    /* static Vector3 calculateDrag(const Vector3& velocity, double altitude, 
                               double dragCoefficient, double referenceArea) {
        double density = getDensity(altitude);
        double velocityMagnitude = velocity.magnitude();
        double dragMagnitude = 0.5 * density * velocityMagnitude * velocityMagnitude * 
                              dragCoefficient * referenceArea;
        
        if (velocityMagnitude > 0) {
            return -velocity.normalized() * dragMagnitude;
        }
        return Vector3(0, 0, 0);
    } */

    static double getSpeedOfSound(double altitude) {
        // Approximation valid for troposphere
        const double temperature = 288.15 - 0.0065 * altitude;  // Kelvin
        return std::sqrt(1.4 * 287.05 * temperature);  // m/s
    }

    static double getMachNumber(const Vector3& velocity, double altitude) {
        return velocity.magnitude() / getSpeedOfSound(altitude);
    }
};
