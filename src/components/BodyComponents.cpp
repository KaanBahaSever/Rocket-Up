#include "rocketup/components/BodyComponents.hpp"

#include <cmath>

#include "JsonHelpers.hpp"
#include "rocketup/core/Io.hpp"

namespace rocketup {

using mathrix::kPi;

const char* toString(NoseShape s) {
    switch (s) {
        case NoseShape::Conical: return "conical";
        case NoseShape::Ogive: return "ogive";
        case NoseShape::Ellipsoid: return "ellipsoid";
        case NoseShape::PowerSeries: return "power";
        case NoseShape::Parabolic: return "parabolic";
        case NoseShape::Haack: return "haack";
    }
    return "ogive";
}

NoseShape noseShapeFromString(const std::string& s) {
    const std::string n = io::toLower(s);
    if (n == "conical" || n == "cone") return NoseShape::Conical;
    if (n == "ogive" || n == "tangent-ogive") return NoseShape::Ogive;
    if (n == "ellipsoid" || n == "elliptical") return NoseShape::Ellipsoid;
    if (n == "power" || n == "power-series") return NoseShape::PowerSeries;
    if (n == "parabolic") return NoseShape::Parabolic;
    if (n == "haack" || n == "von-karman" || n == "vonkarman") return NoseShape::Haack;
    throw RocketUpError("unknown nose shape '" + s + "'");
}

namespace {
/// Radius of a profile of length L and height R at distance x from the tip.
double profileRadius(NoseShape shape, double p, double x, double L, double R) {
    if (L <= 0.0 || R <= 0.0) return std::max(R, 0.0);
    const double xi = mathrix::clamp(x / L, 0.0, 1.0);
    switch (shape) {
        case NoseShape::Conical: return R * xi;
        case NoseShape::Ogive: {
            const double rho = (R * R + L * L) / (2.0 * R);
            const double d = L - xi * L;
            return std::max(0.0, std::sqrt(std::max(0.0, rho * rho - d * d)) + R - rho);
        }
        case NoseShape::Ellipsoid: {
            const double u = 1.0 - xi;
            return R * std::sqrt(std::max(0.0, 1.0 - u * u));
        }
        case NoseShape::PowerSeries: {
            const double n = mathrix::clamp(p, 1e-3, 1.0);
            return R * std::pow(xi, n);
        }
        case NoseShape::Parabolic: {
            const double K = mathrix::clamp(p, 0.0, 1.0);
            return R * (2.0 * xi - K * xi * xi) / (2.0 - K);
        }
        case NoseShape::Haack: {
            const double th = std::acos(mathrix::clamp(1.0 - 2.0 * xi, -1.0, 1.0));
            const double s = std::sin(th);
            const double v = th - std::sin(2.0 * th) / 2.0 + p * s * s * s;
            return R / std::sqrt(kPi) * std::sqrt(std::max(0.0, v));
        }
    }
    return R * xi;
}

constexpr int kStations = 200;

MassProperties tubeMass(double length, double ro, double ri, double density) {
    MassProperties m;
    ri = mathrix::clamp(ri, 0.0, ro);
    m.mass = density * kPi * (ro * ro - ri * ri) * length;
    m.cg = 0.5 * length;
    m.ixx = 0.5 * m.mass * (ro * ro + ri * ri);
    m.iyy = m.mass * ((ro * ro + ri * ri) / 4.0 + length * length / 12.0);
    return m;
}
}  // namespace

double noseProfile(NoseShape shape, double parameter, double xi) {
    // Normalised using a fineness ratio of 3 for shapes that depend on it (ogive).
    return profileRadius(shape, parameter, xi * 3.0, 3.0, 1.0);
}

// ------------------------------------------------------------------ SymmetricComponent

SymmetricComponent::SymmetricComponent(std::string name, double length) : Component(std::move(name)), length_(length) {
    if (length_ < 0.0) throw RocketUpError("component length must be >= 0");
}

double SymmetricComponent::outerRadius() const { return std::max(foreRadius(), aftRadius()); }

bool SymmetricComponent::acceptsChild(const Component& child) const { return !child.isBody(); }

SymmetricComponent& SymmetricComponent::setLength(double l) {
    if (l < 0.0) throw RocketUpError("component length must be >= 0");
    length_ = l;
    return *this;
}

double SymmetricComponent::wettedArea() const {
    double a = 0.0;
    const double dx = length_ / kStations;
    double r0 = radiusAt(0.0);
    for (int i = 1; i <= kStations; ++i) {
        const double r1 = radiusAt(i * dx);
        a += kPi * (r0 + r1) * std::sqrt(dx * dx + (r1 - r0) * (r1 - r0));
        r0 = r1;
    }
    return a;
}

double SymmetricComponent::planformArea() const {
    double a = 0.0;
    const double dx = length_ / kStations;
    double r0 = radiusAt(0.0);
    for (int i = 1; i <= kStations; ++i) {
        const double r1 = radiusAt(i * dx);
        a += (r0 + r1) * dx;
        r0 = r1;
    }
    return a;
}

double SymmetricComponent::planformCentroid() const {
    double a = 0.0, m = 0.0;
    const double dx = length_ / kStations;
    double r0 = radiusAt(0.0);
    for (int i = 1; i <= kStations; ++i) {
        const double r1 = radiusAt(i * dx);
        const double da = (r0 + r1) * dx;
        a += da;
        m += da * (i - 0.5) * dx;
        r0 = r1;
    }
    return a > 0.0 ? m / a : 0.5 * length_;
}

double SymmetricComponent::volume() const {
    double v = 0.0;
    const double dx = length_ / kStations;
    double r0 = radiusAt(0.0);
    for (int i = 1; i <= kStations; ++i) {
        const double r1 = radiusAt(i * dx);
        v += kPi / 3.0 * dx * (r0 * r0 + r0 * r1 + r1 * r1);
        r0 = r1;
    }
    return v;
}

MassProperties SymmetricComponent::computeMass() const {
    MassProperties total;
    const double rho = material_.density;
    if (length_ > 0.0) {
        const double dx = length_ / kStations;
        double r0 = radiusAt(0.0);
        for (int i = 1; i <= kStations; ++i) {
            const double r1 = radiusAt(i * dx);
            const double rm = 0.5 * (r0 + r1);
            const double ds = std::sqrt(dx * dx + (r1 - r0) * (r1 - r0));
            double dV, ri;
            if (filled_ || rm <= thickness_) {
                dV = kPi / 3.0 * dx * (r0 * r0 + r0 * r1 + r1 * r1);
                ri = 0.0;
            } else {
                dV = kPi * ds * thickness_ * (2.0 * rm - thickness_);
                ri = rm - thickness_;
            }
            MassProperties s;
            s.mass = rho * dV;
            s.cg = (i - 0.5) * dx;
            s.ixx = 0.5 * s.mass * (rm * rm + ri * ri);
            s.iyy = s.mass * ((rm * rm + ri * ri) / 4.0 + dx * dx / 12.0);
            total += s;
            r0 = r1;
        }
    }
    total += extraMass();
    return total;
}

void SymmetricComponent::writeCommon(json& j) const {
    j["length"] = length_;
    j["thickness"] = thickness_;
    if (filled_) j["filled"] = true;
    j["material"] = material_.toJson();
    j["finish"] = toString(finish_);
}

void SymmetricComponent::readCommon(const json& j) {
    length_ = j.value("length", length_);
    thickness_ = j.value("thickness", thickness_);
    filled_ = j.value("filled", false);
    if (j.contains("material")) material_ = Material::fromJson(j.at("material"));
    if (j.contains("finish")) finish_ = surfaceFinishFromString(j.at("finish").get<std::string>());
}

// ------------------------------------------------------------------ NoseCone

NoseCone::NoseCone(std::string name, NoseShape shape, double length, double baseRadius, double shapeParameter)
    : SymmetricComponent(std::move(name), length), shape_(shape), baseRadius_(baseRadius), parameter_(shapeParameter) {}

double NoseCone::radiusAt(double x) const { return profileRadius(shape_, parameter_, x, length_, baseRadius_); }

NoseCone& NoseCone::setShoulder(double length, double radius, double thickness) {
    shoulderLength_ = length;
    shoulderRadius_ = radius;
    shoulderThickness_ = thickness;
    return *this;
}

MassProperties NoseCone::extraMass() const {
    if (shoulderLength_ <= 0.0) return {};
    const double ro = shoulderRadius_ > 0.0 ? shoulderRadius_ : baseRadius_ - thickness_;
    const double t = shoulderThickness_ > 0.0 ? shoulderThickness_ : thickness_;
    return tubeMass(shoulderLength_, ro, filled_ ? 0.0 : ro - t, material_.density).shifted(length_);
}

void NoseCone::writeProperties(json& j) const {
    j["shape"] = toString(shape_);
    j["shapeParameter"] = parameter_;
    j["baseRadius"] = baseRadius_;
    writeCommon(j);
    if (shoulderLength_ > 0.0)
        j["shoulder"] = {{"length", shoulderLength_}, {"radius", shoulderRadius_}, {"thickness", shoulderThickness_}};
}

void NoseCone::readProperties(const json& j) {
    readCommon(j);
    if (j.contains("shape")) shape_ = noseShapeFromString(j.at("shape").get<std::string>());
    parameter_ = j.value("shapeParameter", parameter_);
    baseRadius_ = detail::readRadius(j, "baseRadius", "baseDiameter", baseRadius_);
    if (j.contains("shoulder")) {
        const auto& s = j.at("shoulder");
        setShoulder(s.value("length", 0.0), detail::readRadius(s, "radius", "diameter", 0.0), s.value("thickness", 0.0));
    }
}

// ------------------------------------------------------------------ BodyTube

BodyTube::BodyTube(std::string name, double length, double radius, double thickness)
    : SymmetricComponent(std::move(name), length), radius_(radius) {
    thickness_ = thickness;
}

void BodyTube::writeProperties(json& j) const {
    j["radius"] = radius_;
    writeCommon(j);
}

void BodyTube::readProperties(const json& j) {
    readCommon(j);
    radius_ = detail::readRadius(j, "radius", "diameter", radius_);
}

// ------------------------------------------------------------------ Transition

Transition::Transition(std::string name, double length, double foreRadius, double aftRadius, NoseShape shape,
                       double shapeParameter)
    : SymmetricComponent(std::move(name), length), fore_(foreRadius), aft_(aftRadius), shape_(shape),
      parameter_(shapeParameter) {}

double Transition::radiusAt(double x) const {
    if (aft_ >= fore_) return fore_ + profileRadius(shape_, parameter_, x, length_, aft_ - fore_);
    return aft_ + profileRadius(shape_, parameter_, length_ - x, length_, fore_ - aft_);
}

void Transition::writeProperties(json& j) const {
    j["foreRadius"] = fore_;
    j["aftRadius"] = aft_;
    j["shape"] = toString(shape_);
    j["shapeParameter"] = parameter_;
    writeCommon(j);
}

void Transition::readProperties(const json& j) {
    readCommon(j);
    fore_ = detail::readRadius(j, "foreRadius", "foreDiameter", fore_);
    aft_ = detail::readRadius(j, "aftRadius", "aftDiameter", aft_);
    if (j.contains("shape")) shape_ = noseShapeFromString(j.at("shape").get<std::string>());
    parameter_ = j.value("shapeParameter", parameter_);
}

}  // namespace rocketup
