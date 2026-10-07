#pragma once

#include <string>

#include "rocketup/sim/Events.hpp"

namespace rocketup {

/// Flight phase of a body.
enum class FlightPhase { OnPad, OnRail, Powered, Coasting, Descent, Landed };
const char* toString(FlightPhase p);

/// True (noise-free) state of a body at one instant. Positions are in the local ENU frame
/// with the origin at the foot of the launch rail.
struct FlightSnapshot {
    double time = 0.0;           ///< s since launch
    Vec3 position;               ///< m (z = altitude above the launch site)
    Vec3 velocity;               ///< m/s
    Vec3 acceleration;           ///< m/s^2
    double altitude = 0.0;       ///< m above launch site
    double altitudeAsl = 0.0;    ///< m above datum
    double verticalVelocity = 0.0;
    double speed = 0.0;          ///< ground-relative
    double airspeed = 0.0;
    double mach = 0.0;
    double dynamicPressure = 0.0;
    double angleOfAttack = 0.0;  ///< rad
    double mass = 0.0;
    double thrust = 0.0;
    Quat attitude;               ///< body -> ENU
    Vec3 angularVelocity;        ///< body frame, rad/s
    double maxAltitude = 0.0;    ///< so far for this body
    FlightPhase phase = FlightPhase::OnPad;
};

/// Simulated sensor outputs (what a flight computer would see). Accelerometer and gyro are
/// in the body frame: x towards the nose. The accelerometer measures specific force, so it
/// reads +1 g along x while standing on the pad.
struct SensorReadings {
    bool valid = false;
    double time = 0.0;
    double pressure = 0.0;          ///< Pa
    double temperature = 0.0;       ///< K
    double baroAltitude = 0.0;      ///< m above launch site from the barometric formula
    double maxBaroAltitude = 0.0;   ///< running maximum of baroAltitude
    double baroVerticalSpeed = 0.0; ///< low-pass filtered derivative of baroAltitude, m/s
    Vec3 acceleration;              ///< m/s^2 specific force, body frame
    Vec3 angularRate;               ///< rad/s, body frame
    bool gpsFix = false;
    double gpsLatitude = 0.0, gpsLongitude = 0.0, gpsAltitude = 0.0;  ///< deg, deg, m ASL
    Vec3 gpsVelocity;               ///< ENU m/s
};

/// Everything a trigger function can look at.
struct TriggerContext {
    const FlightSnapshot& state;
    const SensorReadings& sensors;
    const EventLog& events;
    const ExternalCommands& external;
    const std::string& body;

    double time() const { return state.time; }
    /// Seconds since the first matching event, or -1 if it has not happened.
    double sinceEvent(EventType t, const std::string& source = "") const {
        const double te = events.timeOf(t, source);
        return te < 0.0 ? -1.0 : state.time - te;
    }
};

}  // namespace rocketup
