#pragma once

#include "rocketup/components/Component.hpp"
#include "rocketup/components/InternalComponents.hpp"
#include "rocketup/components/Material.hpp"
#include "rocketup/sim/Trigger.hpp"

namespace rocketup {

/// Anything that adds drag on command: parachutes, streamers, drogues.
class RecoveryDevice : public Component {
public:
    double length() const override { return packedLength_; }
    double outerRadius() const override { return packedRadius_; }

    const Trigger& deployTrigger() const { return trigger_; }
    RecoveryDevice& setDeployTrigger(Trigger t) { trigger_ = std::move(t); return *this; }
    /// Extra delay between the trigger and the start of deployment (ejection charge delay).
    double deployDelay() const { return delay_; }
    RecoveryDevice& setDeployDelay(double s) { delay_ = s; return *this; }
    /// Time to go from packed to fully open (drag area ramps up quadratically).
    double inflationTime() const { return inflation_; }
    RecoveryDevice& setInflationTime(double s) { inflation_ = s; return *this; }
    RecoveryDevice& setPacked(double length, double radius) {
        packedLength_ = length;
        packedRadius_ = radius;
        return *this;
    }

    /// Fully open drag area Cd*A (m^2).
    virtual double dragArea() const = 0;
    /// Drag area `t` seconds after deployment started.
    double dragAreaAt(double t) const;

protected:
    RecoveryDevice(std::string name);
    RecoveryDevice(const RecoveryDevice&) = default;
    void writeCommon(json& j) const;
    void readCommon(const json& j);

    Trigger trigger_ = Trigger::apogee();
    double delay_ = 0.0;
    double inflation_ = 0.3;
    double packedLength_ = 0.1, packedRadius_ = 0.03;
};

class Parachute final : public RecoveryDevice {
public:
    Parachute(std::string name = "Parachute", double diameter = 1.0, double cd = 0.8);
    ROCKETUP_COMPONENT_CLONE(Parachute)
    const char* type() const override { return "parachute"; }

    double dragArea() const override;
    double diameter() const { return diameter_; }
    Parachute& setDiameter(double d) { diameter_ = d; return *this; }
    double cd() const { return cd_; }
    Parachute& setCd(double cd) { cd_ = cd; return *this; }
    double spillHoleDiameter() const { return spill_; }
    Parachute& setSpillHole(double d) { spill_ = d; return *this; }
    Parachute& setLines(int count, double length, Material material = materials::nylonCord());
    Parachute& setCanopyMaterial(Material m) { canopy_ = std::move(m); return *this; }
    double canopyArea() const;

protected:
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    double diameter_, cd_, spill_ = 0.0;
    Material canopy_ = materials::ripstopNylon();
    int lineCount_ = 6;
    double lineLength_ = 1.0;
    Material line_ = materials::nylonCord();
};

class Streamer final : public RecoveryDevice {
public:
    Streamer(std::string name = "Streamer", double length = 1.0, double width = 0.1, double cd = 0.3);
    ROCKETUP_COMPONENT_CLONE(Streamer)
    const char* type() const override { return "streamer"; }
    double dragArea() const override { return cd_ * length_ * width_; }

protected:
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    double length_, width_, cd_;
    Material material_ = materials::ripstopNylon();
};

/// A payload (CanSat, experiment, ...). It rides inside the rocket until its eject trigger
/// fires; then it becomes an independent body that falls with its own recovery devices
/// (children) and its own drag, so drift and descent rate are computed separately.
class Payload final : public MassObject {
public:
    Payload(std::string name = "Payload", double mass = 1.0, double length = 0.15, double radius = 0.04);
    ROCKETUP_COMPONENT_CLONE(Payload)
    const char* type() const override { return "payload"; }
    bool acceptsChild(const Component& child) const override;

    const Trigger& ejectTrigger() const { return eject_; }
    /// Eject from the rocket when `t` fires (default: never, payload stays attached).
    Payload& setEjectTrigger(Trigger t) { eject_ = std::move(t); return *this; }
    double ejectionSpeed() const { return ejectionSpeed_; }
    Payload& setEjectionSpeed(double v) { ejectionSpeed_ = v; return *this; }
    /// Drag area of the bare payload when free falling (m^2). <= 0: estimated as a tumbling cylinder.
    double freeFallDragArea() const;
    Payload& setFreeFallDragArea(double cda) { cda_ = cda; return *this; }

protected:
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    Trigger eject_ = Trigger::never();
    double ejectionSpeed_ = 2.0;
    double cda_ = -1.0;
};

}  // namespace rocketup
