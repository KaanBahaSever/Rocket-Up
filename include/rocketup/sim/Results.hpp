#pragma once

#include <functional>
#include <string>
#include <vector>

#include "rocketup/sim/Events.hpp"
#include "rocketup/sim/FlightState.hpp"

namespace rocketup {

/// One time sample of a body's flight. Positions/velocities in the local ENU frame.
struct FlightRecord {
    double time = 0.0;
    FlightPhase phase = FlightPhase::OnPad;
    Vec3 position, velocity, acceleration;
    double altitude = 0.0, altitudeAsl = 0.0, latitude = 0.0, longitude = 0.0;
    double speed = 0.0, verticalVelocity = 0.0, horizontalSpeed = 0.0, airspeed = 0.0;
    double mach = 0.0, reynolds = 0.0, dynamicPressure = 0.0, angleOfAttack = 0.0;
    double thrust = 0.0, dragForce = 0.0, axialForce = 0.0, normalForce = 0.0, gravity = 0.0;
    double cd = 0.0, cdFriction = 0.0, cdPressure = 0.0, cdBase = 0.0, cnAlpha = 0.0;
    double cp = 0.0, cg = 0.0, stabilityMargin = 0.0;  ///< cp/cg from nose tip (m), margin in calibers
    double mass = 0.0, propellantMass = 0.0, ixx = 0.0, iyy = 0.0;
    Vec3 angularVelocity;              ///< body frame (roll, pitch, yaw), rad/s
    double zenith = 0.0, azimuth = 0.0;  ///< axis direction: angle from vertical / heading (deg)
    double axialAcceleration = 0.0;    ///< specific force along the axis (what an accelerometer reads), m/s^2
    Vec3 wind;
    double temperature = 0.0, pressure = 0.0, density = 0.0, speedOfSound = 0.0;
    double recoveryDragArea = 0.0, recoveryForce = 0.0;
};

/// Table column description for exporters/GUI (name, unit, accessor).
struct RecordColumn {
    std::string key;    ///< machine name, e.g. "altitude"
    std::string label;  ///< human label, e.g. "Altitude AGL"
    std::string unit;
    std::function<double(const FlightRecord&)> get;
};
const std::vector<RecordColumn>& recordColumns();
const RecordColumn* findRecordColumn(const std::string& key);

/// What happened to one recovery device.
struct RecoveryReport {
    std::string device;
    std::string type;
    double triggerTime = -1.0, deployTime = -1.0;
    double deployAltitude = 0.0, deploySpeed = 0.0;
    double peakForce = 0.0, peakForceTime = 0.0;  ///< opening shock (N)
    double dragArea = 0.0;
};

/// Touchdown of a body.
struct LandingReport {
    bool landed = false;
    double time = 0.0;
    Vec3 position;                 ///< ENU m
    double latitude = 0.0, longitude = 0.0;
    double distance = 0.0;         ///< from the launch rail, m
    double bearing = 0.0;          ///< deg clockwise from north
    double impactSpeed = 0.0;      ///< m/s
    double descentRate = 0.0;      ///< mean vertical speed over the last 100 m, m/s
    double kineticEnergy = 0.0;    ///< J at impact
    double driftFromApogee = 0.0;  ///< horizontal distance between apogee and landing, m
};

/// Full history of one body (the rocket, a separated section or an ejected payload).
struct BodyResult {
    std::string name;
    std::string parent;  ///< body it separated from ("" for the launch vehicle)
    double startTime = 0.0;
    std::vector<FlightRecord> records;
    EventLog events;
    std::vector<RecoveryReport> recovery;
    LandingReport landing;
    double mass = 0.0;  ///< mass at the end of the flight

    double apogee() const;
    double apogeeTime() const;
};

/// Key figures of a simulation (computed from the body histories).
struct FlightSummary {
    double apogee = 0.0, apogeeAsl = 0.0, apogeeTime = 0.0;
    double maxSpeed = 0.0, maxSpeedTime = 0.0, maxMach = 0.0;
    double maxAcceleration = 0.0, maxAxialG = 0.0, maxDynamicPressure = 0.0;
    double railExitSpeed = 0.0, railExitTime = 0.0;
    double burnoutTime = 0.0, burnoutAltitude = 0.0, burnoutSpeed = 0.0;
    double stabilityAtRailExit = 0.0, minStability = 0.0, maxStability = 0.0;
    double maxAngleOfAttack = 0.0;  ///< deg, ascent after rail exit
    double liftoffMass = 0.0, burnoutMass = 0.0, propellantBurned = 0.0;
    double flightTime = 0.0;
    double apogeeRange = 0.0;  ///< horizontal distance of apogee from the pad
    std::vector<std::string> notes;

    json toJson() const;
};

struct SimulationResult {
    std::string rocketName;
    std::vector<BodyResult> bodies;  ///< bodies[0] is the launch vehicle
    std::vector<FlightEvent> events; ///< all events, time ordered
    FlightSummary summary;
    std::vector<std::string> warnings;
    double computeTime = 0.0;  ///< s of wall clock
    long long steps = 0;

    const BodyResult* body(const std::string& name) const;
    /// Index of the body that reached the highest apogee (the sustainer / payload carrier).
    size_t primaryBody() const;
    /// Complete history of a body including its ancestors before it separated from them.
    std::vector<FlightRecord> trajectory(size_t index) const;
    json summaryJson() const;
};

/// Compute the summary from the raw histories (called by the simulation).
FlightSummary computeSummary(const SimulationResult& r);

}  // namespace rocketup
