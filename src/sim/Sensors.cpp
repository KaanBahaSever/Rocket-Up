#include "rocketup/sim/Sensors.hpp"

#include <cmath>
#include <nlohmann/json.hpp>

namespace rocketup {

json SensorConfig::toJson() const {
    return {{"enabled", enabled},
            {"imuRate", imuRate},
            {"gpsRate", gpsRate},
            {"baroNoise", baroNoise},
            {"accelNoise", accelNoise},
            {"accelRange", accelRange},
            {"gyroNoise", gyroNoise},
            {"gyroRange", gyroRange},
            {"gpsHorizontalNoise", gpsHorizontalNoise},
            {"gpsVerticalNoise", gpsVerticalNoise},
            {"baroSpeedTimeConstant", baroSpeedTimeConstant}};
}

SensorConfig SensorConfig::fromJson(const json& j) {
    SensorConfig c;
    c.enabled = j.value("enabled", c.enabled);
    c.imuRate = j.value("imuRate", c.imuRate);
    c.gpsRate = j.value("gpsRate", c.gpsRate);
    c.baroNoise = j.value("baroNoise", c.baroNoise);
    c.accelNoise = j.value("accelNoise", c.accelNoise);
    c.accelRange = j.value("accelRange", c.accelRange);
    c.gyroNoise = j.value("gyroNoise", c.gyroNoise);
    c.gyroRange = j.value("gyroRange", c.gyroRange);
    c.gpsHorizontalNoise = j.value("gpsHorizontalNoise", c.gpsHorizontalNoise);
    c.gpsVerticalNoise = j.value("gpsVerticalNoise", c.gpsVerticalNoise);
    c.baroSpeedTimeConstant = j.value("baroSpeedTimeConstant", c.baroSpeedTimeConstant);
    return c;
}

SensorSimulator::SensorSimulator(SensorConfig config, uint64_t seed) : cfg_(config), rng_(seed) {}

void SensorSimulator::update(double time, const FlightSnapshot& s, const Vec3& f, double pressure, double temperature,
                             double latitude, double longitude) {
    if (!cfg_.enabled) return;
    if (time + 1e-12 >= nextImu_) {
        nextImu_ = time + 1.0 / cfg_.imuRate;
        const double p = std::max(0.0, pressure + cfg_.baroNoise * n_(rng_));
        if (groundPressure_ < 0.0) groundPressure_ = pressure;
        r_.pressure = p;
        r_.temperature = temperature;
        // Standard-atmosphere barometric formula, as typical flight computers use it.
        const double alt = groundPressure_ > 0.0 && p > 0.0 ? 44330.77 * (1.0 - std::pow(p / groundPressure_, 0.190263)) : 0.0;
        if (lastBaroTime_ >= 0.0 && time > lastBaroTime_) {
            const double dt = time - lastBaroTime_;
            const double raw = (alt - lastBaroAlt_) / dt;
            const double a = dt / (cfg_.baroSpeedTimeConstant + dt);
            r_.baroVerticalSpeed += a * (raw - r_.baroVerticalSpeed);
        }
        lastBaroTime_ = time;
        lastBaroAlt_ = alt;
        r_.baroAltitude = alt;
        r_.maxBaroAltitude = std::max(r_.maxBaroAltitude, alt);
        auto clip = [](double v, double lim) { return mathrix::clamp(v, -lim, lim); };
        r_.acceleration = {clip(f.x + cfg_.accelNoise * n_(rng_), cfg_.accelRange),
                           clip(f.y + cfg_.accelNoise * n_(rng_), cfg_.accelRange),
                           clip(f.z + cfg_.accelNoise * n_(rng_), cfg_.accelRange)};
        const Vec3& w = s.angularVelocity;
        r_.angularRate = {clip(w.x + cfg_.gyroNoise * n_(rng_), cfg_.gyroRange),
                          clip(w.y + cfg_.gyroNoise * n_(rng_), cfg_.gyroRange),
                          clip(w.z + cfg_.gyroNoise * n_(rng_), cfg_.gyroRange)};
        r_.time = time;
        r_.valid = true;
    }
    if (time + 1e-12 >= nextGps_) {
        nextGps_ = time + 1.0 / cfg_.gpsRate;
        const double mPerDeg = 111320.0;
        r_.gpsFix = true;
        r_.gpsLatitude = latitude + cfg_.gpsHorizontalNoise * n_(rng_) / mPerDeg;
        r_.gpsLongitude =
            longitude + cfg_.gpsHorizontalNoise * n_(rng_) / (mPerDeg * std::max(0.01, std::cos(mathrix::deg2rad(latitude))));
        r_.gpsAltitude = s.altitudeAsl + cfg_.gpsVerticalNoise * n_(rng_);
        r_.gpsVelocity = s.velocity + Vec3{0.1 * n_(rng_), 0.1 * n_(rng_), 0.2 * n_(rng_)};
    }
}

}  // namespace rocketup
