#include "rocketup/environment/Environment.hpp"

#include <cmath>
#include <nlohmann/json.hpp>

#include "rocketup/core/Io.hpp"

namespace rocketup {

using mathrix::deg2rad;
using mathrix::rad2deg;

json LaunchSite::toJson() const {
    json j = {{"latitude", latitude}, {"longitude", longitude}, {"altitude", altitude}};
    if (!name.empty()) j["name"] = name;
    return j;
}

LaunchSite LaunchSite::fromJson(const json& j) {
    LaunchSite s;
    s.name = j.value("name", std::string());
    s.latitude = j.value("latitude", 0.0);
    s.longitude = j.value("longitude", 0.0);
    s.altitude = j.value("altitude", 0.0);
    return s;
}

Vec3 LaunchRail::direction() const {
    const double el = deg2rad(elevation), az = deg2rad(azimuth);
    return {std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el)};
}

json LaunchRail::toJson() const {
    json j = {{"length", length}, {"elevation", elevation}, {"azimuth", azimuth}};
    if (frictionCoefficient != 0.0) j["friction"] = frictionCoefficient;
    return j;
}

LaunchRail LaunchRail::fromJson(const json& j) {
    LaunchRail r;
    r.length = j.value("length", r.length);
    r.elevation = j.value("elevation", r.elevation);
    r.azimuth = j.value("azimuth", r.azimuth);
    r.frictionCoefficient = j.value("friction", 0.0);
    if (r.elevation <= 0.0 || r.elevation > 90.0) throw RocketUpError("launch rail elevation must be in (0, 90] deg");
    return r;
}

Environment::Environment() : Environment(planets::earth()) {}

Environment::Environment(Planet planet, LaunchSite site, LaunchRail rail, std::unique_ptr<WindModel> wind)
    : planet_(std::move(planet)),
      site_(std::move(site)),
      rail_(rail),
      wind_(wind ? std::move(wind) : std::make_unique<NoWind>()) {}

Environment::Environment(const Environment& o)
    : planet_(o.planet_), site_(o.site_), rail_(o.rail_), wind_(o.wind_->clone()), rotatingFrame_(o.rotatingFrame_) {}

Environment& Environment::operator=(const Environment& o) {
    if (this != &o) {
        Environment tmp(o);
        *this = std::move(tmp);
    }
    return *this;
}

double Environment::gravityAt(double altitudeAgl) const {
    const double h = site_.altitude + altitudeAgl;
    const double g = planet_.gravity(h);
    if (planet_.gravityModel() == Planet::GravityModel::Constant) return g;
    const double w = planet_.rotationRate();
    const double c = std::cos(deg2rad(site_.latitude));
    return g - w * w * (planet_.radius() + h) * c * c;
}

Vec3 Environment::planetRotationEnu() const {
    const double lat = deg2rad(site_.latitude);
    return Vec3{0.0, std::cos(lat), std::sin(lat)} * planet_.rotationRate();
}

void Environment::toGeodetic(const Vec3& enu, double& latitude, double& longitude) const {
    const double R = planet_.radius() + site_.altitude;
    latitude = site_.latitude + rad2deg(enu.y / R);
    const double c = std::cos(deg2rad(site_.latitude));
    longitude = site_.longitude + (std::abs(c) > 1e-9 ? rad2deg(enu.x / (R * c)) : 0.0);
}

json Environment::toJson() const {
    json j = {{"format", "rocketup-environment"},
              {"version", 1},
              {"planet", planet_.toJson()},
              {"site", site_.toJson()},
              {"rail", rail_.toJson()},
              {"wind", wind_->toJson()}};
    if (rotatingFrame_) j["rotatingFrame"] = true;
    return j;
}

Environment Environment::fromJson(const json& j, const std::string& baseDirectory) {
    Planet p = j.contains("planet") ? Planet::fromJson(j.at("planet"), baseDirectory) : planets::earth();
    if (j.contains("conditions")) {
        // Measured launch-site weather: {"temperature": K, "pressure": Pa} (either optional).
        const auto& c = j.at("conditions");
        auto* layered = dynamic_cast<LayeredAtmosphere*>(&p.atmosphere());
        const double alt = j.contains("site") ? j.at("site").value("altitude", 0.0) : 0.0;
        if (layered) {
            const AtmosphereState s = layered->at(alt);
            layered->setConditionsAt(alt, c.value("temperature", s.temperature), c.value("pressure", s.pressure));
        }
    }
    Environment e(std::move(p), j.contains("site") ? LaunchSite::fromJson(j.at("site")) : LaunchSite{},
                  j.contains("rail") ? LaunchRail::fromJson(j.at("rail")) : LaunchRail{},
                  j.contains("wind") ? WindModel::fromJson(j.at("wind"), baseDirectory) : nullptr);
    e.setRotatingFrame(j.value("rotatingFrame", false));
    return e;
}

Environment Environment::loadFile(const std::string& path) {
    return fromJson(io::readJsonFile(path), io::directoryOf(path));
}

void Environment::saveFile(const std::string& path) const { io::writeJsonFile(path, toJson()); }

}  // namespace rocketup
