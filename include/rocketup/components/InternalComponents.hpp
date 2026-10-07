#pragma once

#include <algorithm>
#include <optional>

#include "rocketup/components/Component.hpp"
#include "rocketup/components/Material.hpp"
#include "rocketup/propulsion/Motor.hpp"
#include "rocketup/sim/Trigger.hpp"

namespace rocketup {

/// Inner tube / coupler / avionics bay tube. `outerRadius <= 0` means "fit the parent".
class InnerTube : public Component {
public:
    InnerTube(std::string name = "Inner tube", double length = 0.2, double outerRadius = -1.0,
              double thickness = 0.002);
    ROCKETUP_COMPONENT_CLONE(InnerTube)
    const char* type() const override { return "inner-tube"; }
    double length() const override { return length_; }
    double outerRadius() const override;
    bool acceptsChild(const Component& child) const override;
    InnerTube& setLength(double l) { length_ = l; return *this; }
    double thickness() const { return thickness_; }
    InnerTube& setThickness(double t) { thickness_ = t; return *this; }
    double innerRadius() const;
    const Material& material() const { return material_; }
    InnerTube& setMaterial(Material m) { material_ = std::move(m); return *this; }

protected:
    InnerTube(const InnerTube&) = default;
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

    double length_, outerRadius_, thickness_;
    Material material_ = materials::fiberglass();
};

/// Bulkhead, centering ring or engine block (an annulus/disc). Radii <= 0 mean "auto":
/// outer fits the parent, inner = 0.
class Ring final : public Component {
public:
    Ring(std::string name = "Bulkhead", double length = 0.01, double outerRadius = -1.0, double innerRadius = 0.0);
    ROCKETUP_COMPONENT_CLONE(Ring)
    const char* type() const override { return "ring"; }
    double length() const override { return length_; }
    double outerRadius() const override;
    double innerRadius() const { return std::max(0.0, innerRadius_); }
    const Material& material() const { return material_; }
    Ring& setMaterial(Material m) { material_ = std::move(m); return *this; }
    const std::string& role() const { return role_; }  ///< "bulkhead", "centering-ring", "engine-block"
    Ring& setRole(std::string r) { role_ = std::move(r); return *this; }

protected:
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    double length_, outerRadius_, innerRadius_;
    Material material_ = materials::plywood();
    std::string role_ = "bulkhead";
};

/// Point-like mass: avionics, batteries, ballast, shock cords, hardware...
class MassObject : public Component {
public:
    MassObject(std::string name = "Mass", double mass = 0.1, double length = 0.05, double radius = 0.02,
               std::string category = "generic");
    ROCKETUP_COMPONENT_CLONE(MassObject)
    const char* type() const override { return "mass"; }
    double length() const override { return length_; }
    double outerRadius() const override { return radius_; }
    double massValue() const { return mass_; }
    MassObject& setMass(double m) { mass_ = m; return *this; }
    const std::string& category() const { return category_; }
    MassObject& setCategory(std::string c) { category_ = std::move(c); return *this; }

protected:
    MassObject(const MassObject&) = default;
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

    double mass_, length_, radius_;
    std::string category_;
};

/// Motor tube holding an (optional) motor. The motor's aft end is flush with the mount's
/// aft end plus `overhang`. Ignition defaults to launch; use a trigger for staged or
/// air-started motors.
class MotorMount final : public InnerTube {
public:
    MotorMount(std::string name = "Motor mount", double length = 0.3, double outerRadius = 0.0275,
               double thickness = 0.0015);
    ROCKETUP_COMPONENT_CLONE(MotorMount)
    const char* type() const override { return "motor-mount"; }

    bool hasMotor() const { return motor_.has_value(); }
    const Motor& motor() const;
    MotorMount& setMotor(Motor m) { motor_ = std::move(m); return *this; }
    MotorMount& clearMotor() { motor_.reset(); return *this; }
    double overhang() const { return overhang_; }
    MotorMount& setOverhang(double o) { overhang_ = o; return *this; }
    const Trigger& ignitionTrigger() const { return ignition_; }
    MotorMount& setIgnitionTrigger(Trigger t) { ignition_ = std::move(t); return *this; }

    /// Local axial position of the motor's forward end.
    double motorFront() const;
    /// Motor mass properties `t` seconds after ignition (t < 0: not yet ignited), local frame.
    MassProperties motorMass(double t) const;

protected:
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    std::optional<Motor> motor_;
    double overhang_ = 0.0;
    Trigger ignition_ = Trigger::launch();
};

}  // namespace rocketup
