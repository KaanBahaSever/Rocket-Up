#pragma once

#include <memory>
#include <string>

#include "rocketup/environment/Planet.hpp"
#include "rocketup/environment/Wind.hpp"

namespace rocketup {

/// Geographic launch location.
struct LaunchSite {
    std::string name;
    double latitude = 0.0;   ///< deg
    double longitude = 0.0;  ///< deg
    double altitude = 0.0;   ///< m above planet datum (sea level)

    json toJson() const;
    static LaunchSite fromJson(const json& j);
};

/// Launch rail / rod. The rocket is constrained to slide along it until it has travelled
/// `length` metres.
struct LaunchRail {
    double length = 6.0;      ///< m of guided travel
    double elevation = 85.0;  ///< deg above the horizon (90 = vertical)
    double azimuth = 0.0;     ///< deg clockwise from north (direction the rail points to)
    double frictionCoefficient = 0.0;

    /// Unit vector along the rail in the local ENU frame.
    Vec3 direction() const;
    json toJson() const;
    static LaunchRail fromJson(const json& j);
};

/// Everything outside of the rocket: the planet (gravity + atmosphere), the site, the rail and the wind.
/// The local frame is East-North-Up with its origin at the foot of the rail.
class Environment {
public:
    Environment();
    explicit Environment(Planet planet, LaunchSite site = {}, LaunchRail rail = {},
                         std::unique_ptr<WindModel> wind = nullptr);
    Environment(const Environment& o);
    Environment& operator=(const Environment& o);
    Environment(Environment&&) noexcept = default;
    Environment& operator=(Environment&&) noexcept = default;

    const Planet& planet() const { return planet_; }
    Planet& planet() { return planet_; }
    void setPlanet(Planet p) { planet_ = std::move(p); }

    const LaunchSite& site() const { return site_; }
    LaunchSite& site() { return site_; }
    void setSite(LaunchSite s) { site_ = std::move(s); }

    const LaunchRail& rail() const { return rail_; }
    LaunchRail& rail() { return rail_; }
    void setRail(LaunchRail r) { rail_ = r; }

    const WindModel& wind() const { return *wind_; }
    void setWind(std::unique_ptr<WindModel> w) { wind_ = w ? std::move(w) : std::make_unique<NoWind>(); }

    /// Include the Coriolis acceleration of the rotating planet (off by default; the
    /// centrifugal part is always folded into gravityAt()).
    bool rotatingFrame() const { return rotatingFrame_; }
    void setRotatingFrame(bool on) { rotatingFrame_ = on; }

    // Convenience queries. `altitudeAgl` is the local z coordinate (above the launch site).
    AtmosphereState atmosphereAt(double altitudeAgl) const { return planet_.atmosphere().at(site_.altitude + altitudeAgl); }
    /// Effective (gravitational minus centrifugal) acceleration magnitude at the site latitude.
    double gravityAt(double altitudeAgl) const;
    Vec3 windAt(double altitudeAgl, double time) const { return wind_->velocity(altitudeAgl, time); }
    /// Planet angular velocity expressed in the local ENU frame.
    Vec3 planetRotationEnu() const;
    /// Convert local ENU position to latitude/longitude (deg) using a spherical planet.
    void toGeodetic(const Vec3& enu, double& latitude, double& longitude) const;

    json toJson() const;
    static Environment fromJson(const json& j, const std::string& baseDirectory = "");
    static Environment loadFile(const std::string& path);
    void saveFile(const std::string& path) const;

private:
    Planet planet_;
    LaunchSite site_;
    LaunchRail rail_;
    std::unique_ptr<WindModel> wind_;
    bool rotatingFrame_ = false;
};

}  // namespace rocketup
