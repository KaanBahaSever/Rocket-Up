#include "rocketup/components/Recovery.hpp"

#include <cmath>

#include "JsonHelpers.hpp"

namespace rocketup {

using mathrix::kPi;

// ------------------------------------------------------------------ RecoveryDevice

RecoveryDevice::RecoveryDevice(std::string name) : Component(std::move(name)) {}

double RecoveryDevice::dragAreaAt(double t) const {
    if (t < 0.0) return 0.0;
    const double full = dragArea();
    if (inflation_ <= 0.0 || t >= inflation_) return full;
    const double tau = t / inflation_;
    return full * tau * tau;
}

void RecoveryDevice::writeCommon(json& j) const {
    j["deploy"] = trigger_.toJson();
    if (delay_ != 0.0) j["deployDelay"] = delay_;
    j["inflationTime"] = inflation_;
    j["packed"] = {{"length", packedLength_}, {"radius", packedRadius_}};
}

void RecoveryDevice::readCommon(const json& j) {
    if (j.contains("deploy")) trigger_ = Trigger::fromJson(j.at("deploy"));
    delay_ = j.value("deployDelay", 0.0);
    inflation_ = j.value("inflationTime", inflation_);
    if (j.contains("packed")) {
        packedLength_ = j.at("packed").value("length", packedLength_);
        packedRadius_ = detail::readRadius(j.at("packed"), "radius", "diameter", packedRadius_);
    }
}

// ------------------------------------------------------------------ Parachute

Parachute::Parachute(std::string name, double diameter, double cd)
    : RecoveryDevice(std::move(name)), diameter_(diameter), cd_(cd) {}

double Parachute::canopyArea() const { return kPi / 4.0 * std::max(0.0, diameter_ * diameter_ - spill_ * spill_); }

double Parachute::dragArea() const { return cd_ * canopyArea(); }

Parachute& Parachute::setLines(int count, double length, Material material) {
    lineCount_ = count;
    lineLength_ = length;
    line_ = std::move(material);
    return *this;
}

MassProperties Parachute::computeMass() const {
    MassProperties m;
    m.mass = canopyArea() * canopy_.density + lineCount_ * lineLength_ * line_.density;
    m.cg = 0.5 * packedLength_;
    m.ixx = 0.5 * m.mass * packedRadius_ * packedRadius_;
    m.iyy = m.mass * (packedRadius_ * packedRadius_ / 4.0 + packedLength_ * packedLength_ / 12.0);
    return m;
}

void Parachute::writeProperties(json& j) const {
    j["diameter"] = diameter_;
    j["cd"] = cd_;
    if (spill_ > 0.0) j["spillHoleDiameter"] = spill_;
    j["canopyMaterial"] = canopy_.toJson();
    j["lines"] = {{"count", lineCount_}, {"length", lineLength_}, {"material", line_.toJson()}};
    writeCommon(j);
}

void Parachute::readProperties(const json& j) {
    readCommon(j);
    diameter_ = j.value("diameter", diameter_);
    cd_ = j.value("cd", cd_);
    spill_ = j.value("spillHoleDiameter", 0.0);
    if (j.contains("canopyMaterial")) canopy_ = Material::fromJson(j.at("canopyMaterial"));
    if (j.contains("lines")) {
        const auto& l = j.at("lines");
        lineCount_ = l.value("count", lineCount_);
        lineLength_ = l.value("length", lineLength_);
        if (l.contains("material")) line_ = Material::fromJson(l.at("material"));
    }
}

// ------------------------------------------------------------------ Streamer

Streamer::Streamer(std::string name, double length, double width, double cd)
    : RecoveryDevice(std::move(name)), length_(length), width_(width), cd_(cd) {}

MassProperties Streamer::computeMass() const {
    MassProperties m;
    m.mass = length_ * width_ * material_.density;
    m.cg = 0.5 * packedLength_;
    return m;
}

void Streamer::writeProperties(json& j) const {
    j["length"] = length_;
    j["width"] = width_;
    j["cd"] = cd_;
    j["material"] = material_.toJson();
    writeCommon(j);
}

void Streamer::readProperties(const json& j) {
    readCommon(j);
    length_ = j.value("length", length_);
    width_ = j.value("width", width_);
    cd_ = j.value("cd", cd_);
    if (j.contains("material")) material_ = Material::fromJson(j.at("material"));
}

// ------------------------------------------------------------------ Payload

Payload::Payload(std::string name, double mass, double length, double radius)
    : MassObject(std::move(name), mass, length, radius, "payload") {}

bool Payload::acceptsChild(const Component& child) const {
    return dynamic_cast<const RecoveryDevice*>(&child) != nullptr ||
           (dynamic_cast<const MassObject*>(&child) != nullptr && dynamic_cast<const Payload*>(&child) == nullptr);
}

double Payload::freeFallDragArea() const {
    if (cda_ > 0.0) return cda_;
    const double d = 2.0 * radius_;
    return 0.5 * (1.0 * length_ * d + 0.8 * kPi * d * d / 4.0);
}

void Payload::writeProperties(json& j) const {
    MassObject::writeProperties(j);
    j["eject"] = eject_.toJson();
    j["ejectionSpeed"] = ejectionSpeed_;
    if (cda_ > 0.0) j["freeFallDragArea"] = cda_;
}

void Payload::readProperties(const json& j) {
    MassObject::readProperties(j);
    if (j.contains("eject")) eject_ = Trigger::fromJson(j.at("eject"));
    ejectionSpeed_ = j.value("ejectionSpeed", ejectionSpeed_);
    cda_ = j.value("freeFallDragArea", -1.0);
}

}  // namespace rocketup
