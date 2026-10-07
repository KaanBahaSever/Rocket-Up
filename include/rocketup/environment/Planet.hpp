#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rocketup/environment/Atmosphere.hpp"

namespace rocketup {

/// A celestial body to launch from. Fully data driven: load any planet/moon from JSON.
class Planet {
public:
    enum class GravityModel { InverseSquare, Constant };

    Planet(std::string name, double gravitationalParameter, double radius, std::unique_ptr<AtmosphereModel> atmosphere,
           double rotationRate = 0.0);
    Planet(const Planet& other);
    Planet& operator=(const Planet& other);
    Planet(Planet&&) noexcept = default;
    Planet& operator=(Planet&&) noexcept = default;

    const std::string& name() const { return name_; }
    double gravitationalParameter() const { return mu_; }  ///< GM, m^3/s^2
    double mass() const;                                   ///< kg
    double radius() const { return radius_; }              ///< m
    double rotationRate() const { return rotationRate_; }  ///< rad/s (sidereal)
    double surfaceGravity() const;                         ///< m/s^2

    /// Gravitational acceleration magnitude at geometric altitude above datum.
    double gravity(double altitude) const;

    const AtmosphereModel& atmosphere() const { return *atmosphere_; }
    AtmosphereModel& atmosphere() { return *atmosphere_; }
    void setAtmosphere(std::unique_ptr<AtmosphereModel> a) { atmosphere_ = std::move(a); }

    /// Use a constant gravitational acceleration instead of GM/r^2 (e.g. competition rules).
    void setConstantGravity(double g);
    void setInverseSquareGravity() { gravityModel_ = GravityModel::InverseSquare; }
    GravityModel gravityModel() const { return gravityModel_; }

    const std::string& description() const { return description_; }
    void setDescription(std::string d) { description_ = std::move(d); }

    json toJson() const;
    /// Accepts a full planet object, or a string naming a builtin ("earth", "mars", "moon").
    static Planet fromJson(const json& j, const std::string& baseDirectory = "");
    static Planet loadFile(const std::string& path);
    void saveFile(const std::string& path) const;

private:
    std::string name_;
    std::string description_;
    double mu_;
    double radius_;
    double rotationRate_;
    std::unique_ptr<AtmosphereModel> atmosphere_;
    GravityModel gravityModel_ = GravityModel::InverseSquare;
    double constantGravity_ = 0.0;
};

/// Builtin bodies. Each one is also shipped as an editable JSON file in data/planets/.
namespace planets {
Planet earth();  ///< WGS-84 GM, mean radius, US Standard Atmosphere 1976
Planet mars();   ///< NASA Glenn Mars atmosphere fit (CO2)
Planet moon();   ///< airless
Planet byName(const std::string& name);
std::vector<std::string> builtinNames();
}  // namespace planets

}  // namespace rocketup
