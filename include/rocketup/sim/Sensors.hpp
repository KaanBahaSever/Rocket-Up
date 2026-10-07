#pragma once

#include <cstdint>
#include <random>

#include "rocketup/sim/FlightState.hpp"

namespace rocketup {

/// Noise and sampling characteristics of the simulated flight-computer sensors.
struct SensorConfig {
    bool enabled = true;
    double imuRate = 100.0;          ///< Hz (barometer, accelerometer, gyroscope)
    double gpsRate = 10.0;           ///< Hz
    double baroNoise = 3.0;          ///< Pa (1 sigma); ~0.25 m at sea level
    double accelNoise = 0.08;        ///< m/s^2 (1 sigma)
    double accelRange = 16.0 * 9.80665;  ///< m/s^2 saturation
    double gyroNoise = 0.003;        ///< rad/s (1 sigma)
    double gyroRange = mathrix::deg2rad(2000.0);
    double gpsHorizontalNoise = 2.5; ///< m (1 sigma)
    double gpsVerticalNoise = 5.0;   ///< m (1 sigma)
    double baroSpeedTimeConstant = 0.2;  ///< s, low-pass filter on the barometric vertical speed

    json toJson() const;
    static SensorConfig fromJson(const json& j);
};

/// Generates sensor readings from the true state (sample-and-hold at the sensor rates).
class SensorSimulator {
public:
    SensorSimulator(SensorConfig config = {}, uint64_t seed = 1);

    /// `specificForceBody`: (a - g) in the body frame. `groundPressure` calibrates baro altitude.
    void update(double time, const FlightSnapshot& s, const Vec3& specificForceBody, double pressure,
                double temperature, double latitude, double longitude);
    const SensorReadings& readings() const { return r_; }

private:
    SensorConfig cfg_;
    std::mt19937_64 rng_;
    std::normal_distribution<double> n_{0.0, 1.0};
    SensorReadings r_;
    double nextImu_ = 0.0, nextGps_ = 0.0;
    double groundPressure_ = -1.0;
    double lastBaroTime_ = -1.0, lastBaroAlt_ = 0.0;
};

}  // namespace rocketup
