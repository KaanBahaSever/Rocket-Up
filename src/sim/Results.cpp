#include "rocketup/sim/Results.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

namespace rocketup {

const std::vector<RecordColumn>& recordColumns() {
    using R = FlightRecord;
    static const std::vector<RecordColumn> cols = {
        {"time", "Time", "s", [](const R& r) { return r.time; }},
        {"phase", "Flight phase", "-", [](const R& r) { return static_cast<double>(r.phase); }},
        {"altitude", "Altitude AGL", "m", [](const R& r) { return r.altitude; }},
        {"altitudeAsl", "Altitude ASL", "m", [](const R& r) { return r.altitudeAsl; }},
        {"east", "Position east", "m", [](const R& r) { return r.position.x; }},
        {"north", "Position north", "m", [](const R& r) { return r.position.y; }},
        {"latitude", "Latitude", "deg", [](const R& r) { return r.latitude; }},
        {"longitude", "Longitude", "deg", [](const R& r) { return r.longitude; }},
        {"speed", "Total velocity", "m/s", [](const R& r) { return r.speed; }},
        {"verticalVelocity", "Vertical velocity", "m/s", [](const R& r) { return r.verticalVelocity; }},
        {"horizontalSpeed", "Horizontal velocity", "m/s", [](const R& r) { return r.horizontalSpeed; }},
        {"velocityEast", "Velocity east", "m/s", [](const R& r) { return r.velocity.x; }},
        {"velocityNorth", "Velocity north", "m/s", [](const R& r) { return r.velocity.y; }},
        {"airspeed", "Airspeed", "m/s", [](const R& r) { return r.airspeed; }},
        {"acceleration", "Total acceleration", "m/s^2", [](const R& r) { return r.acceleration.norm(); }},
        {"verticalAcceleration", "Vertical acceleration", "m/s^2", [](const R& r) { return r.acceleration.z; }},
        {"axialAcceleration", "Axial acceleration (accelerometer)", "m/s^2", [](const R& r) { return r.axialAcceleration; }},
        {"mach", "Mach number", "-", [](const R& r) { return r.mach; }},
        {"reynolds", "Reynolds number", "-", [](const R& r) { return r.reynolds; }},
        {"dynamicPressure", "Dynamic pressure", "Pa", [](const R& r) { return r.dynamicPressure; }},
        {"angleOfAttack", "Angle of attack", "deg", [](const R& r) { return mathrix::rad2deg(r.angleOfAttack); }},
        {"thrust", "Thrust", "N", [](const R& r) { return r.thrust; }},
        {"drag", "Drag force", "N", [](const R& r) { return r.dragForce; }},
        {"normalForce", "Normal force", "N", [](const R& r) { return r.normalForce; }},
        {"gravity", "Gravitational acceleration", "m/s^2", [](const R& r) { return r.gravity; }},
        {"cd", "Drag coefficient", "-", [](const R& r) { return r.cd; }},
        {"cdFriction", "Friction drag coefficient", "-", [](const R& r) { return r.cdFriction; }},
        {"cdPressure", "Pressure drag coefficient", "-", [](const R& r) { return r.cdPressure; }},
        {"cdBase", "Base drag coefficient", "-", [](const R& r) { return r.cdBase; }},
        {"cnAlpha", "Normal force coefficient slope", "1/rad", [](const R& r) { return r.cnAlpha; }},
        {"cp", "CP location", "m", [](const R& r) { return r.cp; }},
        {"cg", "CG location", "m", [](const R& r) { return r.cg; }},
        {"stability", "Stability margin", "cal", [](const R& r) { return r.stabilityMargin; }},
        {"mass", "Mass", "kg", [](const R& r) { return r.mass; }},
        {"propellantMass", "Propellant mass", "kg", [](const R& r) { return r.propellantMass; }},
        {"ixx", "Roll moment of inertia", "kg m^2", [](const R& r) { return r.ixx; }},
        {"iyy", "Pitch moment of inertia", "kg m^2", [](const R& r) { return r.iyy; }},
        {"rollRate", "Roll rate", "deg/s", [](const R& r) { return mathrix::rad2deg(r.angularVelocity.x); }},
        {"pitchRate", "Pitch rate", "deg/s", [](const R& r) { return mathrix::rad2deg(r.angularVelocity.y); }},
        {"yawRate", "Yaw rate", "deg/s", [](const R& r) { return mathrix::rad2deg(r.angularVelocity.z); }},
        {"zenith", "Zenith angle", "deg", [](const R& r) { return r.zenith; }},
        {"azimuth", "Azimuth", "deg", [](const R& r) { return r.azimuth; }},
        {"windSpeed", "Wind speed", "m/s", [](const R& r) { return r.wind.norm(); }},
        {"temperature", "Air temperature", "K", [](const R& r) { return r.temperature; }},
        {"pressure", "Air pressure", "Pa", [](const R& r) { return r.pressure; }},
        {"density", "Air density", "kg/m^3", [](const R& r) { return r.density; }},
        {"speedOfSound", "Speed of sound", "m/s", [](const R& r) { return r.speedOfSound; }},
        {"recoveryDragArea", "Recovery drag area", "m^2", [](const R& r) { return r.recoveryDragArea; }},
        {"recoveryForce", "Recovery force", "N", [](const R& r) { return r.recoveryForce; }},
    };
    return cols;
}

const RecordColumn* findRecordColumn(const std::string& key) {
    for (const auto& c : recordColumns())
        if (c.key == key) return &c;
    return nullptr;
}

double BodyResult::apogee() const {
    const FlightEvent* e = events.find(EventType::Apogee);
    if (e && e->body == name) return e->altitude;
    double m = 0.0;
    for (const auto& r : records) m = std::max(m, r.altitude);
    return m;
}

double BodyResult::apogeeTime() const {
    const FlightEvent* e = events.find(EventType::Apogee);
    return e ? e->time : 0.0;
}

const BodyResult* SimulationResult::body(const std::string& name) const {
    for (const auto& b : bodies)
        if (b.name == name) return &b;
    return nullptr;
}

size_t SimulationResult::primaryBody() const {
    size_t best = 0;
    double h = -1e300;
    for (size_t i = 0; i < bodies.size(); ++i) {
        const double a = bodies[i].apogee();
        if (a > h + 1e-6) {
            h = a;
            best = i;
        }
    }
    return best;
}

std::vector<FlightRecord> SimulationResult::trajectory(size_t index) const {
    if (index >= bodies.size()) return {};
    std::vector<const BodyResult*> chain{&bodies[index]};
    for (int guard = 0; guard < 32 && !chain.back()->parent.empty(); ++guard) {
        const BodyResult* p = body(chain.back()->parent);
        if (!p) break;
        chain.push_back(p);
    }
    std::vector<FlightRecord> out;
    for (size_t k = chain.size(); k-- > 0;) {
        const double until = k > 0 ? chain[k - 1]->startTime : 1e300;
        for (const auto& r : chain[k]->records)
            if (r.time < until || (k == 0)) out.push_back(r);
    }
    return out;
}

json FlightSummary::toJson() const {
    return {{"apogee", apogee},
            {"apogeeAsl", apogeeAsl},
            {"apogeeTime", apogeeTime},
            {"apogeeRange", apogeeRange},
            {"maxSpeed", maxSpeed},
            {"maxSpeedTime", maxSpeedTime},
            {"maxMach", maxMach},
            {"maxAcceleration", maxAcceleration},
            {"maxAxialG", maxAxialG},
            {"maxDynamicPressure", maxDynamicPressure},
            {"railExitSpeed", railExitSpeed},
            {"railExitTime", railExitTime},
            {"burnoutTime", burnoutTime},
            {"burnoutAltitude", burnoutAltitude},
            {"burnoutSpeed", burnoutSpeed},
            {"stabilityAtRailExit", stabilityAtRailExit},
            {"minStability", minStability},
            {"maxStability", maxStability},
            {"maxAngleOfAttack", maxAngleOfAttack},
            {"liftoffMass", liftoffMass},
            {"burnoutMass", burnoutMass},
            {"propellantBurned", propellantBurned},
            {"flightTime", flightTime},
            {"notes", notes}};
}

json SimulationResult::summaryJson() const {
    json bodiesJ = json::array();
    for (const auto& b : bodies) {
        json rec = json::array();
        for (const auto& r : b.recovery)
            rec.push_back({{"device", r.device},
                           {"type", r.type},
                           {"deployTime", r.deployTime},
                           {"deployAltitude", r.deployAltitude},
                           {"deploySpeed", r.deploySpeed},
                           {"peakForce", r.peakForce},
                           {"peakForceTime", r.peakForceTime},
                           {"dragArea", r.dragArea}});
        const auto& l = b.landing;
        bodiesJ.push_back({{"name", b.name},
                           {"parent", b.parent},
                           {"startTime", b.startTime},
                           {"apogee", b.apogee()},
                           {"mass", b.mass},
                           {"recovery", rec},
                           {"landing",
                            {{"landed", l.landed},
                             {"time", l.time},
                             {"east", l.position.x},
                             {"north", l.position.y},
                             {"latitude", l.latitude},
                             {"longitude", l.longitude},
                             {"distance", l.distance},
                             {"bearing", l.bearing},
                             {"impactSpeed", l.impactSpeed},
                             {"descentRate", l.descentRate},
                             {"kineticEnergy", l.kineticEnergy},
                             {"driftFromApogee", l.driftFromApogee}}}});
    }
    json ev = json::array();
    for (const auto& e : events) ev.push_back(e.toJson());
    return {{"rocket", rocketName}, {"summary", summary.toJson()}, {"bodies", bodiesJ}, {"events", ev},
            {"warnings", warnings}, {"computeTime", computeTime}, {"steps", steps}};
}

FlightSummary computeSummary(const SimulationResult& r) {
    FlightSummary s;
    if (r.bodies.empty() || r.bodies.front().records.empty()) return s;
    const BodyResult& b = r.bodies[r.primaryBody()];
    const std::vector<FlightRecord> rec = r.trajectory(r.primaryBody());
    if (rec.empty()) return s;
    s.liftoffMass = rec.front().mass;
    const double railExit = b.events.timeOf(EventType::RailExit);
    const double apogeeT = b.events.timeOf(EventType::Apogee);
    double burnout = -1.0;
    for (const auto& e : b.events.all())
        if (e.type == EventType::Burnout) burnout = std::max(burnout, e.time);
    s.minStability = 1e9;
    s.maxStability = -1e9;
    bool stabilitySeen = false;
    for (const auto& x : rec) {
        if (x.speed > s.maxSpeed) {
            s.maxSpeed = x.speed;
            s.maxSpeedTime = x.time;
        }
        s.maxMach = std::max(s.maxMach, x.mach);
        s.maxAcceleration = std::max(s.maxAcceleration, x.acceleration.norm());
        s.maxAxialG = std::max(s.maxAxialG, x.axialAcceleration / 9.80665);
        s.maxDynamicPressure = std::max(s.maxDynamicPressure, x.dynamicPressure);
        const bool ascent = railExit >= 0.0 && x.time >= railExit && (apogeeT < 0.0 || x.time <= apogeeT);
        if (ascent && x.phase != FlightPhase::Descent && x.mach > 0.05 && x.cnAlpha > 0.0) {
            s.minStability = std::min(s.minStability, x.stabilityMargin);
            s.maxStability = std::max(s.maxStability, x.stabilityMargin);
            s.maxAngleOfAttack = std::max(s.maxAngleOfAttack, mathrix::rad2deg(x.angleOfAttack));
            stabilitySeen = true;
        }
    }
    if (!stabilitySeen) s.minStability = s.maxStability = 0.0;
    auto at = [&](double t) -> const FlightRecord& {
        auto it = std::lower_bound(rec.begin(), rec.end(), t, [](const FlightRecord& a, double v) { return a.time < v; });
        if (it == rec.end()) return rec.back();
        return *it;
    };
    if (railExit >= 0.0) {
        const auto& x = at(railExit);
        s.railExitTime = railExit;
        s.railExitSpeed = x.speed;
        s.stabilityAtRailExit = x.stabilityMargin;
    } else {
        s.notes.push_back("rocket never left the launch rail");
    }
    if (burnout >= 0.0) {
        const auto& x = at(burnout);
        s.burnoutTime = burnout;
        s.burnoutAltitude = x.altitude;
        s.burnoutSpeed = x.speed;
        s.burnoutMass = x.mass;
    }
    s.propellantBurned = s.liftoffMass - (burnout >= 0.0 ? s.burnoutMass : rec.back().mass);
    if (const FlightEvent* e = b.events.find(EventType::Apogee)) {
        s.apogee = e->altitude;
        s.apogeeTime = e->time;
        const auto& x = at(e->time);
        s.apogeeAsl = x.altitudeAsl - x.altitude + e->altitude;
        s.apogeeRange = std::hypot(x.position.x, x.position.y);
    } else {
        for (const auto& x : rec) s.apogee = std::max(s.apogee, x.altitude);
    }
    for (const auto& body : r.bodies) s.flightTime = std::max(s.flightTime, body.records.empty() ? 0.0 : body.records.back().time);
    if (s.minStability < 1.0 && stabilitySeen) s.notes.push_back("minimum stability margin below 1 caliber");
    if (s.maxStability > 4.0 && stabilitySeen) s.notes.push_back("stability above 4 calibers: rocket may be over-stable (weathercocking)");
    if (s.railExitSpeed > 0.0 && s.railExitSpeed < 15.0) s.notes.push_back("rail exit velocity below 15 m/s");
    return s;
}

}  // namespace rocketup
