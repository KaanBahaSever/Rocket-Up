#pragma once

#include <string>

#include "rocketup/rocket/Rocket.hpp"

namespace rocketup::io {

struct DrawingOptions {
    double width = 960.0;       ///< px
    bool showInternals = true;  ///< dashed outlines of internal parts, motor, parachutes
    bool showCgCp = true;
    double cg = -1.0;           ///< m from nose; < 0 = loaded CG
    double cp = -1.0;           ///< m from nose; < 0 = CP at Mach 0.3
    bool showDimensions = true;
    bool standalone = true;     ///< emit width/height attributes and an XML-ready root
};

/// Side view of the rocket as an SVG string (colors follow `currentColor`, so it adapts to
/// light and dark themes). Usable for reports, documentation and the future GUI.
std::string drawRocketSvg(const Rocket& rocket, const DrawingOptions& options = {});

}  // namespace rocketup::io
