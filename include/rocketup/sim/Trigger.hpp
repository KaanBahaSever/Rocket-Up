#pragma once

#include <functional>
#include <memory>
#include <string>

#include "rocketup/sim/FlightState.hpp"

namespace rocketup {

/// A condition that fires an action (deploy a parachute, separate a section, ignite a
/// motor, eject a payload). Any C++ callable works:
///
///   chute.setDeployTrigger(Trigger::custom("baro apogee", [](const TriggerContext& c) {
///       return c.sensors.maxBaroAltitude - c.sensors.baroAltitude > 5.0;
///   }));
///
/// Builtin triggers serialize to JSON; custom lambdas serialize by name and can be
/// re-attached after loading through `Trigger::registerNamed`.
class Trigger {
public:
    using Predicate = std::function<bool(const TriggerContext&)>;

    Trigger();  ///< never fires
    Trigger(std::string description, Predicate predicate);

    bool operator()(const TriggerContext& ctx) const { return fn_ ? fn_(ctx) : false; }
    const std::string& description() const { return description_; }
    bool isNever() const { return !fn_; }

    /// Fire `seconds` after the condition first becomes true (the condition is latched).
    Trigger delayed(double seconds) const;
    double delay() const { return delay_; }

    // ---- builtin triggers
    static Trigger never();
    static Trigger launch();
    static Trigger atTime(double seconds);
    static Trigger apogee(double delay = 0.0);
    static Trigger burnout(double delay = 0.0, const std::string& motorMount = "");
    /// Descending through `altitude` (m above launch site) after apogee.
    static Trigger altitudeBelow(double altitude);
    /// Climbing through `altitude` (m above launch site).
    static Trigger altitudeAbove(double altitude);
    static Trigger speedBelow(double speed);
    static Trigger afterEvent(EventType type, double delay = 0.0, const std::string& source = "");
    /// Fires when the external command channel (HIL flight computer, GUI) is set.
    static Trigger external(const std::string& channel);
    /// Sensor based apogee: barometric altitude dropped `drop` m below its maximum.
    static Trigger baroApogee(double drop = 5.0);
    /// Sensor based: barometric altitude below `altitude` after having been above it.
    static Trigger baroAltitudeBelow(double altitude);
    static Trigger custom(const std::string& name, Predicate predicate);
    /// Builtin trigger used instead of this custom one when a design file is loaded in a
    /// program that did not register the custom function (e.g. the command line tool).
    Trigger withFallback(const Trigger& fallback) const;
    /// A trigger previously registered with `registerNamed`.
    static Trigger named(const std::string& name);
    static void registerNamed(const std::string& name, Predicate predicate);

    friend Trigger operator&&(const Trigger& a, const Trigger& b);
    friend Trigger operator||(const Trigger& a, const Trigger& b);
    friend Trigger operator!(const Trigger& a);

    json toJson() const;
    static Trigger fromJson(const json& j);

private:
    std::string description_;
    Predicate fn_;
    std::shared_ptr<const json> spec_;
    double delay_ = 0.0;

    Trigger(std::string description, Predicate predicate, json spec);
};

}  // namespace rocketup
