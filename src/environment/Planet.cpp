#include "rocketup/environment/Planet.hpp"

#include <nlohmann/json.hpp>

#include "rocketup/core/Io.hpp"

namespace rocketup {

Planet::Planet(std::string name, double gravitationalParameter, double radius,
               std::unique_ptr<AtmosphereModel> atmosphere, double rotationRate)
    : name_(std::move(name)),
      mu_(gravitationalParameter),
      radius_(radius),
      rotationRate_(rotationRate),
      atmosphere_(atmosphere ? std::move(atmosphere) : std::make_unique<VacuumAtmosphere>()) {
    if (mu_ <= 0.0 || radius_ <= 0.0) throw RocketUpError("planet '" + name_ + "' needs positive GM and radius");
}

Planet::Planet(const Planet& o)
    : name_(o.name_),
      description_(o.description_),
      mu_(o.mu_),
      radius_(o.radius_),
      rotationRate_(o.rotationRate_),
      atmosphere_(o.atmosphere_->clone()),
      gravityModel_(o.gravityModel_),
      constantGravity_(o.constantGravity_) {}

Planet& Planet::operator=(const Planet& o) {
    if (this != &o) {
        Planet tmp(o);
        *this = std::move(tmp);
    }
    return *this;
}

double Planet::mass() const { return mu_ / constants::kGravitationalConstant; }

double Planet::surfaceGravity() const { return gravity(0.0); }

double Planet::gravity(double altitude) const {
    if (gravityModel_ == GravityModel::Constant) return constantGravity_;
    const double r = radius_ + altitude;
    return mu_ / (r * r);
}

void Planet::setConstantGravity(double g) {
    gravityModel_ = GravityModel::Constant;
    constantGravity_ = g;
}

json Planet::toJson() const {
    json j = {{"format", "rocketup-planet"},
              {"version", 1},
              {"name", name_},
              {"gravitationalParameter", mu_},
              {"radius", radius_},
              {"rotationRate", rotationRate_},
              {"atmosphere", atmosphere_->toJson()}};
    if (!description_.empty()) j["description"] = description_;
    if (gravityModel_ == GravityModel::Constant) j["gravity"] = {{"model", "constant"}, {"value", constantGravity_}};
    return j;
}

Planet Planet::fromJson(const json& j, const std::string& baseDirectory) {
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (io::extensionOf(s) == ".json") return loadFile(io::resolvePath(s, baseDirectory));
        return planets::byName(s);
    }
    if (j.contains("file")) {
        Planet p = loadFile(io::resolvePath(j.at("file").get<std::string>(), baseDirectory));
        if (j.contains("gravity") && j.at("gravity").value("model", "") == "constant")
            p.setConstantGravity(j.at("gravity").at("value").get<double>());
        return p;
    }
    if (j.contains("builtin")) {
        Planet p = planets::byName(j.at("builtin").get<std::string>());
        if (j.contains("gravity") && j.at("gravity").value("model", "") == "constant")
            p.setConstantGravity(j.at("gravity").at("value").get<double>());
        if (j.contains("atmosphere"))
            p.setAtmosphere(AtmosphereModel::fromJson(j.at("atmosphere"), p.radius(), p.surfaceGravity(), baseDirectory));
        return p;
    }
    const std::string name = j.value("name", std::string("Unnamed planet"));
    double mu = 0.0;
    if (j.contains("gravitationalParameter")) mu = j.at("gravitationalParameter").get<double>();
    else if (j.contains("mass")) mu = j.at("mass").get<double>() * constants::kGravitationalConstant;
    else throw RocketUpError("planet '" + name + "' needs 'gravitationalParameter' or 'mass'");
    const double radius = j.at("radius").get<double>();
    const double g0 = mu / (radius * radius);
    std::unique_ptr<AtmosphereModel> atm;
    if (j.contains("atmosphere") && !j.at("atmosphere").is_null())
        atm = AtmosphereModel::fromJson(j.at("atmosphere"), radius, g0, baseDirectory);
    Planet p(name, mu, radius, std::move(atm), j.value("rotationRate", 0.0));
    p.setDescription(j.value("description", std::string()));
    if (j.contains("gravity") && j.at("gravity").value("model", "") == "constant")
        p.setConstantGravity(j.at("gravity").at("value").get<double>());
    return p;
}

Planet Planet::loadFile(const std::string& path) { return fromJson(io::readJsonFile(path), io::directoryOf(path)); }

void Planet::saveFile(const std::string& path) const { io::writeJsonFile(path, toJson()); }

namespace planets {

Planet earth() {
    const double R = 6371008.8;
    Planet p("Earth", 3.986004418e14, R,
             std::make_unique<LayeredAtmosphere>(LayeredAtmosphere::usStandard1976(R)), 7.2921159e-5);
    p.setDescription("WGS-84 GM, IUGG mean radius, US Standard Atmosphere 1976");
    return p;
}

Planet mars() {
    const double R = 3389500.0;
    const double mu = 4.282837e13;
    // NASA Glenn Research Center Mars model: T = -31 - 0.000998 h (h < 7 km), -23.4 - 0.00222 h above.
    std::vector<LayeredAtmosphere::Layer> layers = {{0.0, -0.000998}, {7000.0, -0.00222}};
    auto atm = std::make_unique<LayeredAtmosphere>(GasProperties::carbonDioxide(), 242.15, 699.0, layers, mu / (R * R),
                                                   R, false, 60000.0);
    Planet p("Mars", mu, R, std::move(atm), 7.088218e-5);
    p.setDescription("NASA Glenn Mars atmosphere (CO2), hydrostatically integrated");
    return p;
}

Planet moon() {
    Planet p("Moon", 4.9048695e12, 1737400.0, std::make_unique<VacuumAtmosphere>(), 2.6617e-6);
    p.setDescription("Airless body");
    return p;
}

Planet byName(const std::string& name) {
    const std::string n = io::toLower(name);
    if (n == "earth" || n == "dunya" || n == "world") return earth();
    if (n == "mars") return mars();
    if (n == "moon" || n == "ay" || n == "luna") return moon();
    throw RocketUpError("unknown builtin planet '" + name + "' (load it from a JSON file instead)");
}

std::vector<std::string> builtinNames() { return {"earth", "mars", "moon"}; }

}  // namespace planets
}  // namespace rocketup
