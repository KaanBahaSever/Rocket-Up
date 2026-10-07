#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "rocketup/environment/Environment.hpp"
#include "rocketup/rocket/Rocket.hpp"
#include "rocketup/sim/Simulation.hpp"

namespace rocketup {

/// 1-sigma uncertainties used by the Monte Carlo dispersion analysis.
struct Dispersion {
    double windSpeedFactor = 0.20;     ///< relative sigma of the mean wind
    double windDirection = 15.0;       ///< deg
    double windExtra = 0.5;            ///< m/s random extra wind (any direction)
    double thrust = 0.03;              ///< relative (total impulse scatter)
    double dragCoefficient = 0.05;     ///< relative
    double mass = 0.01;                ///< relative (structure without motor)
    double railElevation = 0.5;        ///< deg
    double railAzimuth = 2.0;          ///< deg
    double parachuteCd = 0.05;         ///< relative (applies to every recovery device)

    json toJson() const;
    static Dispersion fromJson(const json& j);
};

struct MonteCarloOptions {
    int runs = 200;
    int threads = 0;  ///< 0 = hardware concurrency
    uint64_t seed = 2024;
    Dispersion dispersion;
    SimulationOptions simulation;
};

/// Landing statistics of one body.
struct LandingStatistics {
    std::string body;
    std::vector<Vec3> points;  ///< ENU landing points of every run
    Vec3 mean;
    double covEE = 0, covNN = 0, covEN = 0;
    /// Dispersion ellipse semi-axes (m) and orientation (deg from east, CCW) for 1, 2, 3 sigma
    /// equivalent probability (39%, 86%, 99% in 2-D).
    double semiMajor = 0.0, semiMinor = 0.0, angle = 0.0;
    double maxDistance = 0.0;
    double meanImpactSpeed = 0.0;
};

struct MonteCarloRun {
    double apogee = 0.0, maxSpeed = 0.0, railExitSpeed = 0.0, minStability = 0.0;
    std::map<std::string, Vec3> landing;
    double thrustScale = 1.0, cdScale = 1.0, windScale = 1.0, windRotation = 0.0;
};

struct MonteCarloResult {
    std::vector<MonteCarloRun> runs;
    double apogeeMean = 0, apogeeStd = 0, apogeeMin = 0, apogeeMax = 0;
    std::vector<LandingStatistics> landings;
    double computeTime = 0.0;

    json toJson() const;
};

/// Runs many perturbed simulations in parallel and collects apogee and landing dispersion
/// for every body (rocket sections and payloads).
MonteCarloResult runMonteCarlo(const Rocket& rocket, const Environment& env, const MonteCarloOptions& options,
                               const std::function<void(int done, int total)>& progress = {});

/// Interactive HTML page: landing scatter with dispersion ellipses and apogee histogram.
void writeMonteCarloReport(const MonteCarloResult& result, const Rocket& rocket, const Environment& env,
                           const std::string& path);
/// KML with landing points and ellipses for Google Earth.
void writeMonteCarloKml(const MonteCarloResult& result, const Environment& env, const std::string& path);

}  // namespace rocketup
