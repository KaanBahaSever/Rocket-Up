#include "rocketup/sim/Trigger.hpp"

#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <sstream>

namespace rocketup {

namespace {

std::mutex& registryMutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, Trigger::Predicate>& registry() {
    static std::map<std::string, Trigger::Predicate> r;
    return r;
}

std::string num(double v) {
    std::ostringstream s;
    s << v;
    return s.str();
}

}  // namespace

// ------------------------------------------------------------------ events

const char* toString(EventType t) {
    switch (t) {
        case EventType::Launch: return "launch";
        case EventType::Ignition: return "ignition";
        case EventType::Liftoff: return "liftoff";
        case EventType::RailExit: return "rail-exit";
        case EventType::Burnout: return "burnout";
        case EventType::Apogee: return "apogee";
        case EventType::Deployment: return "deployment";
        case EventType::Separation: return "separation";
        case EventType::Ejection: return "ejection";
        case EventType::Landing: return "landing";
        case EventType::Custom: return "custom";
    }
    return "custom";
}

EventType eventTypeFromString(const std::string& s) {
    for (int i = 0; i <= static_cast<int>(EventType::Custom); ++i)
        if (s == toString(static_cast<EventType>(i))) return static_cast<EventType>(i);
    throw RocketUpError("unknown event type '" + s + "'");
}

const char* toString(FlightPhase p) {
    switch (p) {
        case FlightPhase::OnPad: return "pad";
        case FlightPhase::OnRail: return "rail";
        case FlightPhase::Powered: return "powered";
        case FlightPhase::Coasting: return "coast";
        case FlightPhase::Descent: return "descent";
        case FlightPhase::Landed: return "landed";
    }
    return "?";
}

json FlightEvent::toJson() const {
    json j = {{"time", time}, {"type", toString(type)}, {"body", body}, {"altitude", altitude}, {"speed", speed}};
    if (!source.empty()) j["source"] = source;
    if (!message.empty()) j["message"] = message;
    return j;
}

const FlightEvent* EventLog::find(EventType t, const std::string& source) const {
    for (const auto& e : events_)
        if (e.type == t && (source.empty() || e.source == source)) return &e;
    return nullptr;
}

bool EventLog::happened(EventType t, const std::string& source) const { return find(t, source) != nullptr; }

double EventLog::timeOf(EventType t, const std::string& source) const {
    const FlightEvent* e = find(t, source);
    return e ? e->time : -1.0;
}

// ------------------------------------------------------------------ Trigger

Trigger::Trigger() : description_("never"), spec_(std::make_shared<json>(json{{"type", "never"}})) {}

Trigger::Trigger(std::string description, Predicate predicate)
    : Trigger(description, std::move(predicate), json{{"type", "custom"}, {"name", description}}) {}

Trigger::Trigger(std::string description, Predicate predicate, json spec)
    : description_(std::move(description)),
      fn_(std::move(predicate)),
      spec_(std::make_shared<json>(std::move(spec))) {}

Trigger Trigger::delayed(double seconds) const {
    Trigger t = *this;
    t.delay_ = seconds;
    t.description_ = description_ + " + " + num(seconds) + " s";
    return t;
}

Trigger Trigger::never() { return Trigger(); }

Trigger Trigger::launch() {
    return Trigger("launch", [](const TriggerContext&) { return true; }, json{{"type", "launch"}});
}

Trigger Trigger::atTime(double seconds) {
    return Trigger("t >= " + num(seconds) + " s", [seconds](const TriggerContext& c) { return c.time() >= seconds; },
                   json{{"type", "time"}, {"time", seconds}});
}

Trigger Trigger::apogee(double delay) {
    auto t = afterEvent(EventType::Apogee, delay);
    t.description_ = delay > 0 ? "apogee + " + num(delay) + " s" : "apogee";
    *std::const_pointer_cast<json>(t.spec_) = json{{"type", "apogee"}, {"delay", delay}};
    return t;
}

Trigger Trigger::burnout(double delay, const std::string& motorMount) {
    auto t = afterEvent(EventType::Burnout, delay, motorMount);
    t.description_ = "burnout" + (delay > 0 ? " + " + num(delay) + " s" : std::string());
    json spec = {{"type", "burnout"}, {"delay", delay}};
    if (!motorMount.empty()) spec["source"] = motorMount;
    *std::const_pointer_cast<json>(t.spec_) = spec;
    return t;
}

Trigger Trigger::altitudeBelow(double altitude) {
    return Trigger("descending below " + num(altitude) + " m",
                   [altitude](const TriggerContext& c) {
                       return c.events.happened(EventType::Apogee) && c.state.altitude <= altitude;
                   },
                   json{{"type", "altitude-below"}, {"altitude", altitude}});
}

Trigger Trigger::altitudeAbove(double altitude) {
    return Trigger("above " + num(altitude) + " m",
                   [altitude](const TriggerContext& c) { return c.state.altitude >= altitude; },
                   json{{"type", "altitude-above"}, {"altitude", altitude}});
}

Trigger Trigger::speedBelow(double speed) {
    return Trigger("speed below " + num(speed) + " m/s",
                   [speed](const TriggerContext& c) {
                       return c.events.happened(EventType::RailExit) && c.state.speed <= speed;
                   },
                   json{{"type", "speed-below"}, {"speed", speed}});
}

Trigger Trigger::afterEvent(EventType type, double delay, const std::string& source) {
    std::string d = std::string(toString(type)) + (source.empty() ? "" : "(" + source + ")");
    if (delay > 0) d += " + " + num(delay) + " s";
    json spec = {{"type", "event"}, {"event", toString(type)}, {"delay", delay}};
    if (!source.empty()) spec["source"] = source;
    return Trigger(d,
                   [type, delay, source](const TriggerContext& c) {
                       const double s = c.sinceEvent(type, source);
                       return s >= 0.0 && s >= delay - 1e-9;
                   },
                   spec);
}

Trigger Trigger::external(const std::string& channel) {
    return Trigger("external command '" + channel + "'",
                   [channel](const TriggerContext& c) { return c.external.has(channel); },
                   json{{"type", "external"}, {"channel", channel}});
}

Trigger Trigger::baroApogee(double drop) {
    return Trigger("barometric apogee (" + num(drop) + " m drop)",
                   [drop](const TriggerContext& c) {
                       return c.sensors.valid && c.sensors.maxBaroAltitude > 2.0 * drop &&
                              c.sensors.maxBaroAltitude - c.sensors.baroAltitude >= drop;
                   },
                   json{{"type", "baro-apogee"}, {"drop", drop}});
}

Trigger Trigger::baroAltitudeBelow(double altitude) {
    return Trigger("barometric altitude below " + num(altitude) + " m",
                   [altitude](const TriggerContext& c) {
                       return c.sensors.valid && c.sensors.maxBaroAltitude > altitude + 20.0 &&
                              c.sensors.baroAltitude <= altitude;
                   },
                   json{{"type", "baro-altitude-below"}, {"altitude", altitude}});
}

Trigger Trigger::custom(const std::string& name, Predicate predicate) {
    return Trigger(name, std::move(predicate), json{{"type", "custom"}, {"name", name}});
}

Trigger Trigger::withFallback(const Trigger& fallback) const {
    Trigger t = *this;
    json spec = spec_ ? *spec_ : json::object();
    spec["fallback"] = fallback.toJson();
    t.spec_ = std::make_shared<json>(std::move(spec));
    return t;
}

void Trigger::registerNamed(const std::string& name, Predicate predicate) {
    std::lock_guard<std::mutex> lock(registryMutex());
    registry()[name] = std::move(predicate);
}

Trigger Trigger::named(const std::string& name) {
    Predicate p;
    {
        std::lock_guard<std::mutex> lock(registryMutex());
        auto it = registry().find(name);
        if (it == registry().end())
            throw RocketUpError("trigger '" + name + "' is not registered (call Trigger::registerNamed first)");
        p = it->second;
    }
    return Trigger(name, std::move(p), json{{"type", "custom"}, {"name", name}});
}

Trigger operator&&(const Trigger& a, const Trigger& b) {
    if (a.isNever() || b.isNever()) return Trigger::never();
    auto fa = a.fn_, fb = b.fn_;
    return Trigger("(" + a.description_ + ") and (" + b.description_ + ")",
                   [fa, fb](const TriggerContext& c) { return fa(c) && fb(c); },
                   json{{"type", "all"}, {"of", json::array({a.toJson(), b.toJson()})}});
}

Trigger operator||(const Trigger& a, const Trigger& b) {
    if (a.isNever()) return b;
    if (b.isNever()) return a;
    auto fa = a.fn_, fb = b.fn_;
    return Trigger("(" + a.description_ + ") or (" + b.description_ + ")",
                   [fa, fb](const TriggerContext& c) { return fa(c) || fb(c); },
                   json{{"type", "any"}, {"of", json::array({a.toJson(), b.toJson()})}});
}

Trigger operator!(const Trigger& a) {
    auto fa = a.fn_;
    return Trigger("not (" + a.description_ + ")",
                   [fa](const TriggerContext& c) { return !fa || !fa(c); },
                   json{{"type", "not"}, {"of", a.toJson()}});
}

json Trigger::toJson() const {
    json j = spec_ ? *spec_ : json{{"type", "never"}};
    if (delay_ != 0.0) j["latchDelay"] = delay_;
    return j;
}

Trigger Trigger::fromJson(const json& j) {
    if (j.is_null()) return never();
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (s == "apogee") return apogee();
        if (s == "launch") return launch();
        if (s == "burnout") return burnout();
        if (s == "never") return never();
        return named(s);
    }
    const std::string type = j.value("type", std::string("never"));
    Trigger t;
    if (type == "never") t = never();
    else if (type == "launch") t = launch();
    else if (type == "time") t = atTime(j.at("time").get<double>());
    else if (type == "apogee") t = apogee(j.value("delay", 0.0));
    else if (type == "burnout") t = burnout(j.value("delay", 0.0), j.value("source", std::string()));
    else if (type == "altitude-below") t = altitudeBelow(j.at("altitude").get<double>());
    else if (type == "altitude-above") t = altitudeAbove(j.at("altitude").get<double>());
    else if (type == "speed-below") t = speedBelow(j.at("speed").get<double>());
    else if (type == "event")
        t = afterEvent(eventTypeFromString(j.at("event").get<std::string>()), j.value("delay", 0.0),
                       j.value("source", std::string()));
    else if (type == "external") t = external(j.at("channel").get<std::string>());
    else if (type == "baro-apogee") t = baroApogee(j.value("drop", 5.0));
    else if (type == "baro-altitude-below") t = baroAltitudeBelow(j.at("altitude").get<double>());
    else if (type == "custom") {
        const std::string name = j.at("name").get<std::string>();
        bool registered;
        {
            std::lock_guard<std::mutex> lock(registryMutex());
            registered = registry().count(name) > 0;
        }
        if (registered) {
            t = named(name);
            json spec = j;
            spec.erase("latchDelay");
            t.spec_ = std::make_shared<json>(std::move(spec));
        } else if (j.contains("fallback")) {
            t = fromJson(j.at("fallback"));
            t.description_ = t.description_ + " (fallback for '" + name + "')";
            json spec = j;
            spec.erase("latchDelay");
            t.spec_ = std::make_shared<json>(std::move(spec));
        } else {
            t = named(name);  // throws a descriptive error
        }
    }
    else if (type == "all" || type == "any") {
        const auto& of = j.at("of");
        if (of.empty()) throw RocketUpError("trigger '" + type + "' needs at least one operand");
        t = fromJson(of.at(0));
        for (size_t i = 1; i < of.size(); ++i) t = type == "all" ? (t && fromJson(of.at(i))) : (t || fromJson(of.at(i)));
    } else if (type == "not")
        t = !fromJson(j.at("of"));
    else
        throw RocketUpError("unknown trigger type '" + type + "'");
    if (j.contains("latchDelay")) t = t.delayed(j.at("latchDelay").get<double>());
    return t;
}

}  // namespace rocketup
