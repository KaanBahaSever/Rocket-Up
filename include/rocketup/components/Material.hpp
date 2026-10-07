#pragma once

#include <string>
#include <vector>

#include "rocketup/core/Types.hpp"

namespace rocketup {

/// A construction material. Bulk materials use kg/m^3, surface materials (fabrics) kg/m^2
/// and line materials (cords) kg/m.
struct Material {
    enum class Kind { Bulk, Surface, Line };
    std::string name = "Fiberglass";
    double density = 1850.0;
    Kind kind = Kind::Bulk;

    json toJson() const;
    /// Accepts a library name ("aluminum") or {"name":..., "density":..., "kind":...}.
    static Material fromJson(const json& j);
};

namespace materials {
Material fiberglass();
Material carbonFiber();
Material aluminum();
Material cardboard();
Material kraftPhenolic();
Material blueTube();
Material plywood();
Material balsa();
Material pla();
Material abs();
Material petg();
Material steel();
Material ripstopNylon();  ///< surface
Material kevlarCord();    ///< line
Material nylonCord();     ///< line
/// Find a library material by (case-insensitive) name; throws if unknown.
Material byName(const std::string& name);
std::vector<Material> library();
}  // namespace materials

/// Surface finish of an external component; sets the roughness height used by the skin
/// friction model (values from Niskanen 2009 / OpenRocket).
enum class SurfaceFinish { Polished, Smooth, Regular, Unfinished, Rough };
double roughnessHeight(SurfaceFinish f);  ///< m
const char* toString(SurfaceFinish f);
SurfaceFinish surfaceFinishFromString(const std::string& s);

}  // namespace rocketup
