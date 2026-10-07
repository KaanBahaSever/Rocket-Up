#pragma once

#include <string>

#include "rocketup/environment/Environment.hpp"
#include "rocketup/rocket/Rocket.hpp"
#include "rocketup/sim/Simulation.hpp"

namespace rocketup {

/// Everything needed to reproduce a flight: rocket design + environment + simulation
/// options + output settings. Stored as JSON (`*.rup.json`); the rocket and the
/// environment may be inline objects or paths to their own files.
struct Project {
    std::string name;
    Rocket rocket;
    Environment environment;
    SimulationOptions options;
    std::string outputDirectory = "output";
    double csvInterval = 0.0;

    json toJson() const;
    static Project fromJson(const json& j, const std::string& baseDirectory = "");
    static Project load(const std::string& path);
    void save(const std::string& path) const;

    /// Accepts a project file, or a bare rocket design (`.rocketup`) with default environment.
    static Project loadAny(const std::string& path);
};

}  // namespace rocketup
