#pragma once

#include <set>
#include <string>
#include <vector>

#include "rocketup/core/Types.hpp"

namespace rocketup {

enum class EventType {
    Launch,      ///< t = 0, simulation start
    Ignition,    ///< a motor ignited (source = motor mount name)
    Liftoff,     ///< rocket starts moving
    RailExit,    ///< left the launch rail
    Burnout,     ///< a motor finished burning (source = motor mount name)
    Apogee,      ///< highest point of a body
    Deployment,  ///< recovery device deployed (source = device name)
    Separation,  ///< body split (source = separation name)
    Ejection,    ///< payload ejected (source = payload name)
    Landing,     ///< body hit the ground
    Custom,      ///< user defined
};

const char* toString(EventType t);
EventType eventTypeFromString(const std::string& s);

struct FlightEvent {
    double time = 0.0;
    EventType type = EventType::Custom;
    std::string body;    ///< body (flight section) the event belongs to
    std::string source;  ///< component / separation name if relevant
    std::string message;
    double altitude = 0.0;  ///< m AGL at the event
    double speed = 0.0;     ///< m/s at the event

    json toJson() const;
};

/// Ordered list of events seen by one body (bodies inherit the history of their parent).
class EventLog {
public:
    void add(FlightEvent e) { events_.push_back(std::move(e)); }
    const std::vector<FlightEvent>& all() const { return events_; }
    bool happened(EventType t, const std::string& source = "") const;
    /// Time of the first matching event, or -1.
    double timeOf(EventType t, const std::string& source = "") const;
    const FlightEvent* find(EventType t, const std::string& source = "") const;

private:
    std::vector<FlightEvent> events_;
};

/// Commands received from outside the simulation (hardware-in-the-loop flight computer,
/// GUI buttons, ...). A channel stays "fired" once received.
class ExternalCommands {
public:
    void fire(const std::string& channel) { fired_.insert(channel); }
    bool has(const std::string& channel) const { return fired_.count(channel) > 0; }
    void clear() { fired_.clear(); }
    const std::set<std::string>& all() const { return fired_; }

private:
    std::set<std::string> fired_;
};

}  // namespace rocketup
