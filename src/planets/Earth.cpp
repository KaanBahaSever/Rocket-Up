#include "Earth.hpp"

// Layer boundaries [m]
static constexpr std::array<double, 7> heights = {
        0, 11000, 20000, 32000, 47000, 51000, 71000};

// Base temperatures [K]
static constexpr std::array<double, 7> baseTemperatures = {
        288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65};
        
// Temperature gradients [K/m]
static constexpr std::array<double, 7> lapse_rates = {
        -0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002};

// Base pressures [Pa]
static constexpr std::array<double, 7> basePressures = {
        101325.0, 22632.1, 5474.89, 868.019, 110.906, 66.9389, 3.95642};

// US Standard Atmosphere 1976 model constants
static constexpr double R = 287.05287; // Specific gas constant [m²/(s²·K)]
static constexpr double g0 = 9.80665;  // Gravitational acceleration at sea level [m/s²]
static double P0 = 101325.0; // Pressure at sea level [Pa]
static double T0 = 288.15;             // Temperature at sea level [K]
static constexpr double rho0 = 1.225;  // Density at sea level [kg/m³]

const double EARTH_MASS = 5.972e24;  // kg
const double EARTH_RADIUS = 6371000; // m
const double EARTH_GRAVITY = 9.81;   // m/s^2
const double EARTH_ATM_DENSITY = 1.225; // kg/m^3

size_t Earth::findLayer(double altitude)
{
    if (altitude <= heights[1])
        return 0;
    if (altitude <= heights[2])
        return 1;
    if (altitude <= heights[3])
        return 2;
    if (altitude <= heights[4])
        return 3;
    if (altitude <= heights[5])
        return 4;
    if (altitude <= heights[6])
        return 5;
    return 6;
}

void Earth::setSeaLevelTemperature(double tempCelsius)
{
    T0 = tempCelsius + 273.15; // Convert Celsius to Kelvin
}

void Earth::setSeaLevelPressure(double pressure)
{
    P0 = pressure;
}

double Earth::temperature(double altitude) const
{
    size_t layer = findLayer(altitude);
    double delta_h = altitude - heights[layer];
    return baseTemperatures[layer] + lapse_rates[layer] * delta_h;
}
double Earth::getTemperature(double altitude) const {
    return temperature(altitude);
}

double Earth::pressure(double altitude) const
{
    size_t layer = findLayer(altitude);
    double delta_h = altitude - heights[layer];
    if (fabs(lapse_rates[layer]) < 1e-10)
    {
        return basePressures[layer] * exp(-g0 * delta_h / (R * baseTemperatures[layer]));
    }
    double temp_ratio = 1 + lapse_rates[layer] * delta_h / baseTemperatures[layer];
    return basePressures[layer] * pow(temp_ratio, -g0 / (R * lapse_rates[layer]));
}

double Earth::density(double altitude) const
{
    return this->pressure(altitude) / (R * temperature(altitude));
}

double Earth::speed_of_sound(double altitude)
{
    // Use ideal gas law: a = sqrt(γRT)
    return sqrt(1.4 * R * temperature(altitude));
}

double Earth::getMass() const {
    return EARTH_MASS;
}

double Earth::getRadius() const {
    return EARTH_RADIUS;
}

double Earth::getGravity() const {
    return EARTH_GRAVITY;
}

double Earth::getAtmosphereDensity() const {
    return EARTH_ATM_DENSITY;
}

std::string Earth::getName() const {
    return "Earth";
}
