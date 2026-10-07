#include "rocketup/propulsion/Motor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <nlohmann/json.hpp>
#include <sstream>

#include "rocketup/core/Io.hpp"

namespace rocketup {

using constants::kStandardGravity;

const char* toString(MassFlowModel m) {
    switch (m) {
        case MassFlowModel::ImpulseProportional: return "impulse-proportional";
        case MassFlowModel::ConstantIsp: return "constant-isp";
        case MassFlowModel::Table: return "table";
    }
    return "impulse-proportional";
}

MassFlowModel massFlowModelFromString(const std::string& s) {
    if (s == "impulse-proportional" || s == "impulse") return MassFlowModel::ImpulseProportional;
    if (s == "constant-isp" || s == "isp") return MassFlowModel::ConstantIsp;
    if (s == "table") return MassFlowModel::Table;
    throw RocketUpError("unknown mass flow model '" + s + "'");
}

namespace {
std::string fmt(double v, int prec = 4) {
    std::ostringstream ss;
    ss.precision(prec);
    ss << v;
    return ss.str();
}
}  // namespace

void Motor::finalize() {
    warnings_.clear();
    if (time.size() != thrust.size() || time.empty())
        throw RocketUpError("motor '" + designation + "': thrust curve is empty or time/thrust sizes differ");

    // Sort, clamp negative thrust, ensure a (0, 0) start point.
    std::vector<std::pair<double, double>> pts;
    for (size_t i = 0; i < time.size(); ++i) {
        if (!std::isfinite(time[i]) || !std::isfinite(thrust[i])) continue;
        if (thrust[i] < 0.0) warnings_.push_back("negative thrust at t=" + fmt(time[i]) + " s clamped to 0");
        pts.emplace_back(time[i], std::max(0.0, thrust[i]));
    }
    std::stable_sort(pts.begin(), pts.end(), [](auto& a, auto& b) { return a.first < b.first; });
    if (pts.empty()) throw RocketUpError("motor '" + designation + "': no valid thrust points");
    if (pts.front().first > 0.0) pts.insert(pts.begin(), {0.0, 0.0});
    time.clear();
    thrust.clear();
    for (auto& [t, f] : pts) {
        time.push_back(t);
        thrust.push_back(f);
    }
    if (time.size() < 2) throw RocketUpError("motor '" + designation + "': thrust curve needs at least 2 points");
    if (thrust.back() > 0.0)
        warnings_.push_back("thrust curve does not end at zero thrust (last point " + fmt(thrust.back()) + " N)");

    curve_ = mathrix::Interpolator1D(time, thrust, interpolation, mathrix::Extrapolation::Zero);
    burnTime_ = time.back();
    totalImpulse_ = curve_.integrate(0.0, burnTime_);
    maxThrust_ = *std::max_element(thrust.begin(), thrust.end());
    if (totalImpulse_ <= 0.0) throw RocketUpError("motor '" + designation + "': total impulse is zero");

    // ---- complete the mass model
    if (propellantMass <= 0.0 && isp > 0.0) {
        propellantMass = totalImpulse_ / (isp * kStandardGravity);
        warnings_.push_back("propellant mass derived from Isp: " + fmt(propellantMass) + " kg");
    } else if (propellantMass <= 0.0) {
        isp = 200.0;
        propellantMass = totalImpulse_ / (isp * kStandardGravity);
        warnings_.push_back("neither propellant mass nor Isp given: assumed Isp = 200 s (APCP) -> propellant " +
                            fmt(propellantMass) + " kg");
    }
    if (totalMass <= 0.0) {
        totalMass = propellantMass / 0.55;
        warnings_.push_back("loaded motor mass unknown: assumed propellant mass fraction 0.55 -> " + fmt(totalMass) +
                            " kg");
    }
    if (totalMass < propellantMass)
        throw RocketUpError("motor '" + designation + "': total mass is smaller than propellant mass");
    if (isp > 0.0) {
        const double ratio = effectiveIsp() / isp;
        if (std::abs(ratio - 1.0) > 0.05 && massModel != MassFlowModel::ConstantIsp)
            warnings_.push_back("stated Isp " + fmt(isp) + " s differs from impulse/propellant Isp " +
                                fmt(effectiveIsp()) + " s");
    }
    if (massModel == MassFlowModel::Table) {
        if (massTime.size() < 2 || massTime.size() != massPropellant.size()) {
            warnings_.push_back("no propellant-mass table: using impulse-proportional consumption");
            massModel = MassFlowModel::ImpulseProportional;
        } else {
            massCurve_ = mathrix::Interpolator1D(massTime, massPropellant, mathrix::Interp::Linear,
                                                mathrix::Extrapolation::Clamp);
        }
    } else if (massTime.size() >= 2 && massTime.size() == massPropellant.size()) {
        massCurve_ = mathrix::Interpolator1D(massTime, massPropellant, mathrix::Interp::Linear,
                                            mathrix::Extrapolation::Clamp);
    }
    if (diameter <= 0.0) warnings_.push_back("motor diameter unknown (base drag / drawing may be off)");
    if (length <= 0.0) warnings_.push_back("motor length unknown (CG position may be off)");
    if (commonName.empty()) {
        // Pull "M2020" out of "8429M2020-P" style designations.
        const std::string d = designation;
        for (size_t i = 0; i < d.size(); ++i) {
            if (std::isalpha(static_cast<unsigned char>(d[i])) && i + 1 < d.size() &&
                std::isdigit(static_cast<unsigned char>(d[i + 1]))) {
                size_t e = i + 1;
                while (e < d.size() && std::isdigit(static_cast<unsigned char>(d[e]))) ++e;
                commonName = d.substr(i, e - i);
                break;
            }
        }
    }
}

Motor& Motor::setInterpolation(Interp method) {
    interpolation = method;
    finalize();
    return *this;
}

Motor& Motor::setMassModel(MassFlowModel model) {
    massModel = model;
    finalize();
    return *this;
}

double Motor::effectiveIsp() const {
    return propellantMass > 0.0 ? totalImpulse() / (propellantMass * kStandardGravity) : 0.0;
}

std::string Motor::impulseClass() const {
    const double I = totalImpulse();
    if (I <= 0.0) return "?";
    if (I <= 0.3125) return "1/4A";
    if (I <= 0.625) return "1/2A";
    const int idx = std::max(0, static_cast<int>(std::ceil(std::log2(I / 2.5) - 1e-12)));
    if (idx > 25) return "Z+";
    return std::string(1, static_cast<char>('A' + idx));
}

std::string Motor::displayName() const {
    std::string s = manufacturer.empty() ? "" : manufacturer + " ";
    return s + (designation.empty() ? commonName : designation);
}

double Motor::thrustAt(double t, double ambientPressure) const {
    if (t < 0.0 || t > burnTime_) return 0.0;
    double f = std::max(0.0, curve_(t));
    if (nozzleExitArea > 0.0 && maxThrust_ > 0.0) {
        // Pressure thrust correction, faded out with the tail-off of the curve.
        const double fade = std::clamp(f / (0.1 * maxThrust_), 0.0, 1.0);
        f += fade * (testPressure - ambientPressure) * nozzleExitArea;
    }
    return std::max(0.0, f * thrustScale);
}

double Motor::impulseAt(double t) const {
    if (t <= 0.0) return 0.0;
    return std::max(0.0, curve_.integrate(0.0, std::min(t, burnTime_)));
}

double Motor::propellantMassAt(double t) const {
    if (t <= 0.0) return propellantMass;
    switch (massModel) {
        case MassFlowModel::ImpulseProportional:
            if (t >= burnTime_) return 0.0;
            return std::max(0.0, propellantMass * (1.0 - impulseAt(t) / totalImpulse_));
        case MassFlowModel::ConstantIsp: {
            const double i = isp > 0.0 ? isp : effectiveIsp();
            return std::max(0.0, propellantMass - impulseAt(t) * thrustScale / (i * kStandardGravity));
        }
        case MassFlowModel::Table: return std::clamp(massCurve_(t), 0.0, propellantMass);
    }
    return 0.0;
}

double Motor::massFlowAt(double t) const {
    if (t < 0.0 || t > burnTime_) return 0.0;
    switch (massModel) {
        case MassFlowModel::ImpulseProportional: return propellantMass * std::max(0.0, curve_(t)) / totalImpulse_;
        case MassFlowModel::ConstantIsp: {
            if (propellantMassAt(t) <= 0.0) return 0.0;
            const double i = isp > 0.0 ? isp : effectiveIsp();
            return std::max(0.0, curve_(t)) * thrustScale / (i * kStandardGravity);
        }
        case MassFlowModel::Table: return std::max(0.0, -massCurve_.derivative(t));
    }
    return 0.0;
}

double Motor::cgAt(double) const { return cgFromTop >= 0.0 ? cgFromTop : 0.5 * length; }

json Motor::toJson() const {
    json j = {{"format", "rocketup-motor"},
              {"version", 1},
              {"designation", designation},
              {"commonName", commonName},
              {"manufacturer", manufacturer},
              {"propellant", propellant},
              {"type", motorType},
              {"diameter", diameter},
              {"length", length},
              {"propellantMass", propellantMass},
              {"totalMass", totalMass},
              {"delays", delays},
              {"massModel", toString(massModel)},
              {"thrustCurve", {{"interpolation", mathrix::toString(interpolation)}, {"time", time}, {"thrust", thrust}}},
              {"summary",
               {{"totalImpulse", totalImpulse()},
                {"burnTime", burnTime()},
                {"averageThrust", averageThrust()},
                {"maxThrust", maxThrust()},
                {"effectiveIsp", effectiveIsp()},
                {"class", impulseClass()}}}};
    if (isp > 0.0) j["isp"] = isp;
    if (nozzleExitArea > 0.0) {
        j["nozzleExitDiameter"] = 2.0 * std::sqrt(nozzleExitArea / mathrix::kPi);
        j["testPressure"] = testPressure;
    }
    if (cgFromTop >= 0.0) j["cgFromTop"] = cgFromTop;
    if (!massTime.empty()) j["propellantMassCurve"] = {{"time", massTime}, {"mass", massPropellant}};
    json src = json::object();
    if (!source.url.empty()) src["url"] = source.url;
    if (!source.contributor.empty()) src["contributor"] = source.contributor;
    if (!source.license.empty()) src["license"] = source.license;
    if (!source.importedFrom.empty()) src["importedFrom"] = source.importedFrom;
    if (!source.notes.empty()) src["notes"] = source.notes;
    if (!src.empty()) j["source"] = src;
    return j;
}

Motor Motor::fromJson(const json& j) {
    Motor m;
    m.designation = j.value("designation", std::string());
    m.commonName = j.value("commonName", std::string());
    m.manufacturer = j.value("manufacturer", std::string());
    m.propellant = j.value("propellant", std::string());
    m.motorType = j.value("type", std::string("reload"));
    m.diameter = j.value("diameter", 0.0);
    m.length = j.value("length", 0.0);
    m.propellantMass = j.value("propellantMass", 0.0);
    m.totalMass = j.value("totalMass", 0.0);
    m.isp = j.value("isp", 0.0);
    if (j.contains("delays")) m.delays = j.at("delays").get<std::vector<std::string>>();
    if (j.contains("nozzleExitDiameter")) {
        const double d = j.at("nozzleExitDiameter").get<double>();
        m.nozzleExitArea = mathrix::kPi * d * d / 4.0;
    }
    m.testPressure = j.value("testPressure", constants::kSeaLevelPressure);
    m.cgFromTop = j.value("cgFromTop", -1.0);
    m.massModel = massFlowModelFromString(j.value("massModel", std::string("impulse-proportional")));
    m.thrustScale = j.value("thrustScale", 1.0);
    const auto& tc = j.at("thrustCurve");
    m.interpolation = mathrix::interpFromString(tc.value("interpolation", std::string("linear")));
    if (tc.contains("points")) {
        for (const auto& p : tc.at("points")) {
            m.time.push_back(p.at(0).get<double>());
            m.thrust.push_back(p.at(1).get<double>());
        }
    } else {
        m.time = tc.at("time").get<std::vector<double>>();
        m.thrust = tc.at("thrust").get<std::vector<double>>();
    }
    if (j.contains("propellantMassCurve")) {
        m.massTime = j.at("propellantMassCurve").at("time").get<std::vector<double>>();
        m.massPropellant = j.at("propellantMassCurve").at("mass").get<std::vector<double>>();
    }
    if (j.contains("source")) {
        const auto& s = j.at("source");
        m.source.url = s.value("url", std::string());
        m.source.contributor = s.value("contributor", std::string());
        m.source.license = s.value("license", std::string());
        m.source.importedFrom = s.value("importedFrom", std::string());
        m.source.notes = s.value("notes", std::string());
    }
    m.finalize();
    return m;
}

}  // namespace rocketup
