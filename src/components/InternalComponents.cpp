#include "rocketup/components/InternalComponents.hpp"

#include <cmath>

#include "JsonHelpers.hpp"
#include "rocketup/components/BodyComponents.hpp"
#include "rocketup/components/Recovery.hpp"
#include "rocketup/core/Io.hpp"

namespace rocketup {

using mathrix::kPi;

namespace {
/// Radius available inside `parent` at local station x.
double innerRadiusOf(const Component* parent, double x) {
    if (!parent) return 0.0;
    if (auto* bt = dynamic_cast<const BodyTube*>(parent)) return bt->innerRadius();
    if (auto* it = dynamic_cast<const InnerTube*>(parent)) return it->innerRadius();
    if (auto* s = dynamic_cast<const SymmetricComponent*>(parent)) {
        const double r = s->radiusAt(mathrix::clamp(x, 0.0, s->length()));
        return s->filled() ? r : std::max(0.0, r - s->thickness());
    }
    return parent->outerRadius();
}

MassProperties cylinder(double mass, double length, double ro, double ri = 0.0) {
    MassProperties m;
    m.mass = mass;
    m.cg = 0.5 * length;
    m.ixx = 0.5 * mass * (ro * ro + ri * ri);
    m.iyy = mass * ((ro * ro + ri * ri) / 4.0 + length * length / 12.0);
    return m;
}
}  // namespace

// ------------------------------------------------------------------ InnerTube

InnerTube::InnerTube(std::string name, double length, double outerRadius, double thickness)
    : Component(std::move(name)), length_(length), outerRadius_(outerRadius), thickness_(thickness) {}

double InnerTube::outerRadius() const {
    if (outerRadius_ > 0.0) return outerRadius_;
    return innerRadiusOf(parent(), positionInParent() + 0.5 * length_);
}

double InnerTube::innerRadius() const { return std::max(0.0, outerRadius() - thickness_); }

bool InnerTube::acceptsChild(const Component& child) const { return !child.isExternal() && !child.isBody(); }

MassProperties InnerTube::computeMass() const {
    const double ro = outerRadius(), ri = innerRadius();
    return cylinder(material_.density * kPi * (ro * ro - ri * ri) * length_, length_, ro, ri);
}

void InnerTube::writeProperties(json& j) const {
    j["length"] = length_;
    j["outerRadius"] = outerRadius_ > 0.0 ? json(outerRadius_) : json("auto");
    j["thickness"] = thickness_;
    j["material"] = material_.toJson();
}

void InnerTube::readProperties(const json& j) {
    length_ = j.value("length", length_);
    if (j.contains("outerRadius") && j.at("outerRadius").is_string()) outerRadius_ = -1.0;
    else outerRadius_ = detail::readRadius(j, "outerRadius", "outerDiameter", outerRadius_);
    thickness_ = j.value("thickness", thickness_);
    if (j.contains("material")) material_ = Material::fromJson(j.at("material"));
}

// ------------------------------------------------------------------ Ring

Ring::Ring(std::string name, double length, double outerRadius, double innerRadius)
    : Component(std::move(name)), length_(length), outerRadius_(outerRadius), innerRadius_(innerRadius) {}

double Ring::outerRadius() const {
    if (outerRadius_ > 0.0) return outerRadius_;
    return innerRadiusOf(parent(), positionInParent() + 0.5 * length_);
}

MassProperties Ring::computeMass() const {
    const double ro = outerRadius(), ri = std::min(innerRadius(), ro);
    return cylinder(material_.density * kPi * (ro * ro - ri * ri) * length_, length_, ro, ri);
}

void Ring::writeProperties(json& j) const {
    j["role"] = role_;
    j["length"] = length_;
    j["outerRadius"] = outerRadius_ > 0.0 ? json(outerRadius_) : json("auto");
    j["innerRadius"] = innerRadius_;
    j["material"] = material_.toJson();
}

void Ring::readProperties(const json& j) {
    role_ = j.value("role", role_);
    length_ = j.value("length", length_);
    if (j.contains("outerRadius") && j.at("outerRadius").is_string()) outerRadius_ = -1.0;
    else outerRadius_ = detail::readRadius(j, "outerRadius", "outerDiameter", outerRadius_);
    innerRadius_ = detail::readRadius(j, "innerRadius", "innerDiameter", innerRadius_);
    if (j.contains("material")) material_ = Material::fromJson(j.at("material"));
}

// ------------------------------------------------------------------ MassObject

MassObject::MassObject(std::string name, double mass, double length, double radius, std::string category)
    : Component(std::move(name)), mass_(mass), length_(length), radius_(radius), category_(std::move(category)) {}

MassProperties MassObject::computeMass() const { return cylinder(mass_, length_, radius_); }

void MassObject::writeProperties(json& j) const {
    j["mass"] = mass_;
    j["length"] = length_;
    j["radius"] = radius_;
    j["category"] = category_;
}

void MassObject::readProperties(const json& j) {
    mass_ = j.value("mass", mass_);
    length_ = j.value("length", length_);
    radius_ = detail::readRadius(j, "radius", "diameter", radius_);
    category_ = j.value("category", category_);
}

// ------------------------------------------------------------------ MotorMount

MotorMount::MotorMount(std::string name, double length, double outerRadius, double thickness)
    : InnerTube(std::move(name), length, outerRadius, thickness) {}

const Motor& MotorMount::motor() const {
    if (!motor_) throw RocketUpError("motor mount '" + name() + "' has no motor");
    return *motor_;
}

double MotorMount::motorFront() const { return motor_ ? length_ - motor_->length + overhang_ : 0.0; }

MassProperties MotorMount::motorMass(double t) const {
    if (!motor_) return {};
    const Motor& m = *motor_;
    const double mass = t < 0.0 ? m.totalMass : m.massAt(t);
    const double r = 0.5 * m.diameter;
    MassProperties p = cylinder(mass, m.length, r);
    p.cg = motorFront() + m.cgAt(std::max(t, 0.0));
    return p;
}

void MotorMount::writeProperties(json& j) const {
    InnerTube::writeProperties(j);
    if (motor_) j["motor"] = motor_->toJson();
    if (overhang_ != 0.0) j["overhang"] = overhang_;
    j["ignition"] = ignition_.toJson();
}

void MotorMount::readProperties(const json& j) {
    InnerTube::readProperties(j);
    overhang_ = j.value("overhang", 0.0);
    if (j.contains("motor")) {
        const auto& mj = j.at("motor");
        if (mj.is_string()) {
            motor_ = MotorLoader::load(io::resolvePath(mj.get<std::string>(), detail::currentBaseDirectory()));
        } else if (mj.contains("file")) {
            MotorLoadOptions o;
            if (mj.contains("propellantMass")) o.propellantMass = mj.at("propellantMass").get<double>();
            if (mj.contains("totalMass")) o.totalMass = mj.at("totalMass").get<double>();
            if (mj.contains("isp")) o.isp = mj.at("isp").get<double>();
            if (mj.contains("interpolation"))
                o.interpolation = mathrix::interpFromString(mj.at("interpolation").get<std::string>());
            if (mj.contains("massModel")) o.massModel = massFlowModelFromString(mj.at("massModel").get<std::string>());
            motor_ = MotorLoader::load(io::resolvePath(mj.at("file").get<std::string>(), detail::currentBaseDirectory()),
                                       o);
        } else {
            motor_ = Motor::fromJson(mj);
        }
    }
    if (j.contains("ignition")) ignition_ = Trigger::fromJson(j.at("ignition"));
}

}  // namespace rocketup
