#include "rocketup/environment/Atmosphere.hpp"

#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>

#include "rocketup/core/Io.hpp"

namespace rocketup {

using constants::kUniversalGasConstant;

double AtmosphereState::kinematicViscosity() const {
    return density > 0.0 ? dynamicViscosity / density : std::numeric_limits<double>::infinity();
}

// ---------------------------------------------------------------- GasProperties

double GasProperties::specificGasConstant() const { return kUniversalGasConstant / molarMass; }

double GasProperties::viscosity(double T) const {
    if (T <= 0.0) return sutherlandMu0;
    return sutherlandMu0 * std::pow(T / sutherlandT0, 1.5) * (sutherlandT0 + sutherlandS) / (T + sutherlandS);
}

double GasProperties::speedOfSound(double T) const { return std::sqrt(gamma * specificGasConstant() * std::max(T, 1.0)); }

double GasProperties::density(double p, double T) const {
    return T > 0.0 ? p / (specificGasConstant() * T) : 0.0;
}

json GasProperties::toJson() const {
    return {{"molarMass", molarMass},
            {"gamma", gamma},
            {"sutherland", {{"mu0", sutherlandMu0}, {"T0", sutherlandT0}, {"S", sutherlandS}}}};
}

GasProperties GasProperties::fromJson(const json& j) {
    GasProperties g;
    if (j.is_string()) {
        const std::string n = io::toLower(j.get<std::string>());
        if (n == "air") return air();
        if (n == "co2" || n == "carbon dioxide") return carbonDioxide();
        if (n == "n2" || n == "nitrogen") return nitrogen();
        throw RocketUpError("unknown gas '" + n + "' (use air, co2, n2 or an object)");
    }
    g.molarMass = j.value("molarMass", g.molarMass);
    g.gamma = j.value("gamma", g.gamma);
    if (j.contains("sutherland")) {
        const auto& s = j.at("sutherland");
        g.sutherlandMu0 = s.value("mu0", g.sutherlandMu0);
        g.sutherlandT0 = s.value("T0", g.sutherlandT0);
        g.sutherlandS = s.value("S", g.sutherlandS);
    }
    return g;
}

GasProperties GasProperties::air() { return {}; }

GasProperties GasProperties::carbonDioxide() {
    GasProperties g;
    g.molarMass = 0.04401;
    g.gamma = 1.29;
    g.sutherlandMu0 = 1.370e-5;
    g.sutherlandT0 = 273.15;
    g.sutherlandS = 222.0;
    return g;
}

GasProperties GasProperties::nitrogen() {
    GasProperties g;
    g.molarMass = 0.0280134;
    g.gamma = 1.4;
    g.sutherlandMu0 = 1.663e-5;
    g.sutherlandT0 = 273.15;
    g.sutherlandS = 107.0;
    return g;
}

// ---------------------------------------------------------------- factory

std::unique_ptr<AtmosphereModel> AtmosphereModel::fromJson(const json& j, double planetRadius, double surfaceGravity,
                                                           const std::string& baseDirectory) {
    const std::string type = j.value("type", std::string("layered"));
    const GasProperties gas = j.contains("gas") ? GasProperties::fromJson(j.at("gas")) : GasProperties::air();
    if (type == "vacuum" || type == "none") return std::make_unique<VacuumAtmosphere>();
    if (type == "us1976" || type == "isa") {
        auto a = std::make_unique<LayeredAtmosphere>(LayeredAtmosphere::usStandard1976(planetRadius));
        if (j.contains("temperatureOffset")) a->setTemperatureOffset(j.at("temperatureOffset").get<double>());
        return a;
    }
    if (type == "layered") {
        std::vector<LayeredAtmosphere::Layer> layers;
        for (const auto& l : j.at("layers"))
            layers.push_back({l.at("baseAltitude").get<double>(), l.at("lapseRate").get<double>()});
        const double g0 = j.value("referenceGravity", surfaceGravity);
        auto a = std::make_unique<LayeredAtmosphere>(gas, j.at("surfaceTemperature").get<double>(),
                                                     j.at("surfacePressure").get<double>(), layers, g0, planetRadius,
                                                     j.value("geopotential", true), j.value("topAltitude", 1e30));
        if (j.contains("temperatureOffset")) a->setTemperatureOffset(j.at("temperatureOffset").get<double>());
        return a;
    }
    if (type == "exponential") {
        return std::make_unique<ExponentialAtmosphere>(gas, j.at("surfacePressure").get<double>(),
                                                       j.at("temperature").get<double>(),
                                                       j.at("scaleHeight").get<double>());
    }
    if (type == "table") {
        if (j.contains("file")) return std::make_unique<TableAtmosphere>(TableAtmosphere::fromCsv(
                                    io::resolvePath(j.at("file").get<std::string>(), baseDirectory), gas));
        return std::make_unique<TableAtmosphere>(
            gas, j.at("altitude").get<std::vector<double>>(), j.at("temperature").get<std::vector<double>>(),
            j.at("pressure").get<std::vector<double>>(), j.value("density", std::vector<double>{}));
    }
    throw RocketUpError("unknown atmosphere type '" + type + "'");
}

// ---------------------------------------------------------------- LayeredAtmosphere

LayeredAtmosphere::LayeredAtmosphere(GasProperties gas, double baseTemperature, double basePressure,
                                     std::vector<Layer> layers, double referenceGravity, double planetRadius,
                                     bool useGeopotential, double topAltitude)
    : gas_(gas),
      baseTemperature_(baseTemperature),
      basePressure_(basePressure),
      layers_(std::move(layers)),
      g0_(referenceGravity),
      radius_(planetRadius),
      geopotential_(useGeopotential),
      top_(topAltitude) {
    if (layers_.empty()) layers_.push_back({0.0, 0.0});
    for (size_t i = 1; i < layers_.size(); ++i)
        if (layers_[i].baseAltitude <= layers_[i - 1].baseAltitude)
            throw RocketUpError("atmosphere layers must have increasing base altitudes");
    rebuild();
}

LayeredAtmosphere LayeredAtmosphere::usStandard1976(double planetRadius) {
    std::vector<Layer> layers = {{0.0, -0.0065},     {11000.0, 0.0},     {20000.0, 0.0010}, {32000.0, 0.0028},
                                 {47000.0, 0.0},     {51000.0, -0.0028}, {71000.0, -0.0020}};
    GasProperties air = GasProperties::air();
    air.molarMass = 0.0289644;
    return LayeredAtmosphere(air, 288.15, 101325.0, layers, 9.80665, planetRadius, true, 84852.0);
}

void LayeredAtmosphere::rebuild() {
    const size_t n = layers_.size();
    layerT_.assign(n, 0.0);
    layerP_.assign(n, 0.0);
    layerT_[0] = baseTemperature_ + temperatureOffset_;
    layerP_[0] = basePressure_;
    const double gMR = g0_ * gas_.molarMass / kUniversalGasConstant;
    for (size_t i = 1; i < n; ++i) {
        const double dh = layers_[i].baseAltitude - layers_[i - 1].baseAltitude;
        const double L = layers_[i - 1].lapseRate;
        const double Tb = layerT_[i - 1];
        const double T = Tb + L * dh;
        layerT_[i] = T;
        if (std::abs(L) < 1e-12) layerP_[i] = layerP_[i - 1] * std::exp(-gMR * dh / Tb);
        else layerP_[i] = layerP_[i - 1] * std::pow(Tb / T, gMR / L);
    }
}

double LayeredAtmosphere::toModelAltitude(double z) const {
    if (!geopotential_ || radius_ <= 0.0) return z;
    return radius_ * z / (radius_ + z);
}

void LayeredAtmosphere::stateAtModelAltitude(double h, double& T, double& p) const {
    const double gMR = g0_ * gas_.molarMass / kUniversalGasConstant;
    const double hTop = toModelAltitude(top_);
    double hEval = std::min(h, hTop);
    size_t i = 0;
    while (i + 1 < layers_.size() && hEval >= layers_[i + 1].baseAltitude) ++i;
    const double dh = hEval - layers_[i].baseAltitude;
    const double L = layers_[i].lapseRate;
    const double Tb = layerT_[i];
    T = std::max(Tb + L * dh, 1.0);
    if (std::abs(L) < 1e-12) p = layerP_[i] * std::exp(-gMR * dh / Tb);
    else p = layerP_[i] * std::pow(Tb / T, gMR / L);
    if (h > hTop) {  // isothermal continuation above the model top
        p *= std::exp(-gMR * (h - hTop) / T);
    }
}

AtmosphereState LayeredAtmosphere::at(double altitude) const {
    AtmosphereState s;
    stateAtModelAltitude(toModelAltitude(altitude), s.temperature, s.pressure);
    s.density = gas_.density(s.pressure, s.temperature);
    s.speedOfSound = gas_.speedOfSound(s.temperature);
    s.dynamicViscosity = gas_.viscosity(s.temperature);
    return s;
}

void LayeredAtmosphere::setTemperatureOffset(double deltaKelvin) {
    temperatureOffset_ = deltaKelvin;
    rebuild();
}

void LayeredAtmosphere::setConditionsAt(double altitude, double temperature, double pressure) {
    // Temperature: shift the profile. Pressure: scale the base pressure (ratios only depend on T).
    temperatureOffset_ = 0.0;
    rebuild();
    double T, p;
    stateAtModelAltitude(toModelAltitude(altitude), T, p);
    temperatureOffset_ = temperature - T;
    rebuild();
    stateAtModelAltitude(toModelAltitude(altitude), T, p);
    basePressure_ *= pressure / p;
    rebuild();
}

json LayeredAtmosphere::toJson() const {
    json layers = json::array();
    for (const auto& l : layers_) layers.push_back({{"baseAltitude", l.baseAltitude}, {"lapseRate", l.lapseRate}});
    json j = {{"type", "layered"},
              {"gas", gas_.toJson()},
              {"surfaceTemperature", baseTemperature_},
              {"surfacePressure", basePressure_},
              {"referenceGravity", g0_},
              {"geopotential", geopotential_},
              {"layers", layers}};
    if (top_ < 1e29) j["topAltitude"] = top_;
    if (temperatureOffset_ != 0.0) j["temperatureOffset"] = temperatureOffset_;
    return j;
}

// ---------------------------------------------------------------- TableAtmosphere

TableAtmosphere::TableAtmosphere(GasProperties gas, std::vector<double> altitude, std::vector<double> temperature,
                                 std::vector<double> pressure, std::vector<double> density)
    : gas_(gas), h_(std::move(altitude)), T_(std::move(temperature)), p_(std::move(pressure)), rho_(std::move(density)) {
    if (h_.size() < 2 || h_.size() != T_.size() || h_.size() != p_.size())
        throw RocketUpError("table atmosphere needs >= 2 rows of altitude/temperature/pressure");
    std::vector<double> logp(p_.size());
    for (size_t i = 0; i < p_.size(); ++i) logp[i] = std::log(std::max(p_[i], 1e-300));
    temperature_ = mathrix::Interpolator1D(h_, T_, mathrix::Interp::Linear, mathrix::Extrapolation::Clamp);
    logPressure_ = mathrix::Interpolator1D(h_, logp, mathrix::Interp::Linear, mathrix::Extrapolation::Linear);
    if (!rho_.empty()) {
        if (rho_.size() != h_.size()) throw RocketUpError("table atmosphere density column size mismatch");
        std::vector<double> logr(rho_.size());
        for (size_t i = 0; i < rho_.size(); ++i) logr[i] = std::log(std::max(rho_[i], 1e-300));
        density_ = mathrix::Interpolator1D(h_, logr, mathrix::Interp::Linear, mathrix::Extrapolation::Linear);
    }
}

TableAtmosphere TableAtmosphere::fromCsv(const std::string& path, GasProperties gas) {
    const auto t = io::readCsvFile(path);
    if (t.columnCount() < 3) throw RocketUpError("atmosphere CSV '" + path + "' needs >= 3 columns");
    return TableAtmosphere(gas, t.column(0), t.column(1), t.column(2),
                           t.columnCount() >= 4 ? t.column(3) : std::vector<double>{});
}

AtmosphereState TableAtmosphere::at(double altitude) const {
    AtmosphereState s;
    s.temperature = temperature_(altitude);
    s.pressure = std::exp(logPressure_(altitude));
    s.density = density_.empty() ? gas_.density(s.pressure, s.temperature) : std::exp(density_(altitude));
    s.speedOfSound = gas_.speedOfSound(s.temperature);
    s.dynamicViscosity = gas_.viscosity(s.temperature);
    return s;
}

json TableAtmosphere::toJson() const {
    json j = {{"type", "table"}, {"gas", gas_.toJson()}, {"altitude", h_}, {"temperature", T_}, {"pressure", p_}};
    if (!rho_.empty()) j["density"] = rho_;
    return j;
}

// ---------------------------------------------------------------- Exponential / Vacuum

ExponentialAtmosphere::ExponentialAtmosphere(GasProperties gas, double surfacePressure, double temperature,
                                             double scaleHeight)
    : gas_(gas), p0_(surfacePressure), T_(temperature), H_(scaleHeight) {
    if (H_ <= 0.0) throw RocketUpError("exponential atmosphere scale height must be positive");
}

AtmosphereState ExponentialAtmosphere::at(double altitude) const {
    AtmosphereState s;
    s.temperature = T_;
    s.pressure = p0_ * std::exp(-altitude / H_);
    s.density = gas_.density(s.pressure, T_);
    s.speedOfSound = gas_.speedOfSound(T_);
    s.dynamicViscosity = gas_.viscosity(T_);
    return s;
}

json ExponentialAtmosphere::toJson() const {
    return {{"type", "exponential"},
            {"gas", gas_.toJson()},
            {"surfacePressure", p0_},
            {"temperature", T_},
            {"scaleHeight", H_}};
}

AtmosphereState VacuumAtmosphere::at(double) const {
    AtmosphereState s;
    s.temperature = 250.0;
    s.pressure = 0.0;
    s.density = 0.0;
    s.speedOfSound = 300.0;
    s.dynamicViscosity = 1e-5;
    return s;
}

json VacuumAtmosphere::toJson() const { return {{"type", "vacuum"}}; }

}  // namespace rocketup
