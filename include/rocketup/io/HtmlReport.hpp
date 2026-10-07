#pragma once

#include <string>

#include "rocketup/environment/Environment.hpp"
#include "rocketup/rocket/Rocket.hpp"
#include "rocketup/sim/Results.hpp"

namespace rocketup::io {

struct ReportOptions {
    std::string title;                 ///< default: "<rocket> flight report"
    int maxPointsPerBody = 2000;       ///< downsampling for the embedded charts
    /// Plotly is loaded from a CDN; set to a local path for offline viewing.
    std::string plotlyUrl = "https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js";
};

/// Self-contained interactive HTML flight report: key figures, rocket drawing, event,
/// recovery and landing tables and ~15 zoomable charts (altitude, velocity, acceleration,
/// forces, Cd breakdown vs Mach, stability, attitude, 3-D trajectory, ground track with
/// all landing points, descent rates, parachute loads, atmosphere, wind, thrust curve).
std::string htmlReport(const SimulationResult& result, const Rocket& rocket, const Environment& env,
                       const ReportOptions& options = {});
void writeHtmlReport(const SimulationResult& result, const Rocket& rocket, const Environment& env,
                     const std::string& path, const ReportOptions& options = {});

/// The Rocket-Up logo as an SVG string.
const std::string& logoSvg();

}  // namespace rocketup::io
