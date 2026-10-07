#pragma once

#include <ostream>
#include <string>
#include <vector>

#include "rocketup/environment/Environment.hpp"
#include "rocketup/rocket/Rocket.hpp"
#include "rocketup/sim/Results.hpp"

namespace rocketup::io {

struct CsvOptions {
    std::vector<std::string> columns;  ///< record column keys (empty = all), see recordColumns()
    double interval = 0.0;             ///< s between rows (0 = every stored record)
    char delimiter = ',';
    int precision = 7;
    bool unitsInHeader = true;         ///< "Altitude AGL (m)" instead of "altitude"
};

/// One CSV per body.
void writeCsv(const BodyResult& body, const std::string& path, const CsvOptions& options = {});
/// Event list as CSV.
void writeEventsCsv(const SimulationResult& result, const std::string& path);
/// Summary, events, landings and recovery data as JSON.
void writeSummaryJson(const SimulationResult& result, const std::string& path);
/// Google Earth KML: 3-D trajectories of every body, events and landing points.
void writeKml(const SimulationResult& result, const Environment& env, const std::string& path);
/// Human readable summary (console).
void printSummary(std::ostream& os, const SimulationResult& result);
/// Rocket design summary: mass, CG, CP, stability, motor.
void printRocketSummary(std::ostream& os, const Rocket& rocket);

/// Write everything (CSV per body, events, summary JSON, KML, HTML report) into a folder.
/// Returns the list of written files.
std::vector<std::string> exportAll(const SimulationResult& result, const Rocket& rocket, const Environment& env,
                                   const std::string& directory);

/// File-system friendly version of a name ("Orbit Chaser" -> "orbit_chaser").
std::string slug(const std::string& name);

}  // namespace rocketup::io
