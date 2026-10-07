#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "rocketup/environment/Environment.hpp"
#include "rocketup/rocket/Rocket.hpp"
#include "rocketup/sim/Results.hpp"
#include "rocketup/sim/Sensors.hpp"

namespace rocketup {

/// Equations of motion used for the powered/coasting flight of the rocket.
enum class DynamicsModel {
    SixDof,    ///< rigid body: translation + rotation (weathercocking, AoA, damping)
    ThreeDof,  ///< point mass, axis always aligned with the relative wind (competition 3-DOF)
};
const char* toString(DynamicsModel d);
DynamicsModel dynamicsModelFromString(const std::string& s);

struct SimulationOptions {
    mathrix::Integrator integrator = mathrix::Integrator::RK4;
    DynamicsModel dynamics = DynamicsModel::SixDof;
    double timeStep = 0.01;      ///< s (initial step for the adaptive integrator)
    double minTimeStep = 1e-5;   ///< adaptive only
    double maxTimeStep = 0.05;   ///< adaptive only
    double absoluteTolerance = 1e-5;
    double relativeTolerance = 1e-6;
    double maxTime = 3600.0;     ///< s
    bool stopAtApogee = false;   ///< end the simulation at the launch vehicle's apogee
    double recordInterval = 0.0; ///< s between stored samples (0 = every step)
    double descentRecordInterval = 0.05;  ///< s between stored samples under parachute / tumbling
    SensorConfig sensors;
    uint64_t seed = 1;

    json toJson() const;
    static SimulationOptions fromJson(const json& j);
};

class Simulation;

/// Hooks into a running simulation: live plots, logging, hardware-in-the-loop...
class SimulationObserver {
public:
    virtual ~SimulationObserver() = default;
    virtual void onStart(const Simulation&) {}
    /// Called after every integration step of every body.
    virtual void onStep(const std::string& body, const FlightRecord& record, const SensorReadings& sensors) {
        (void)body, (void)record, (void)sensors;
    }
    virtual void onEvent(const FlightEvent&) {}
    virtual void onFinish(const SimulationResult&) {}
    /// Called before triggers are evaluated; may fire external command channels.
    virtual void poll(double time, ExternalCommands& commands) { (void)time, (void)commands; }
};

/// Runs a flight. The rocket and environment are copied, so the originals can be edited
/// (e.g. by a GUI) while simulations run in other threads.
class Simulation {
public:
    Simulation(Rocket rocket, Environment environment, SimulationOptions options = {});
    ~Simulation();
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    const Rocket& rocket() const { return rocket_; }
    const Environment& environment() const { return environment_; }
    const SimulationOptions& options() const { return options_; }
    SimulationOptions& options() { return options_; }

    void addObserver(std::shared_ptr<SimulationObserver> observer);
    /// Called periodically with (time, altitude) of the body being integrated; return false to cancel.
    void setProgressCallback(std::function<bool(double, double)> callback);
    /// Thread-safe cancel request.
    void cancel() { cancel_ = true; }

    SimulationResult run();

private:
    Rocket rocket_;
    Environment environment_;
    SimulationOptions options_;
    std::vector<std::shared_ptr<SimulationObserver>> observers_;
    std::function<bool(double, double)> progress_;
    std::atomic<bool> cancel_{false};
};

/// Convenience: simulate with default options.
SimulationResult simulate(const Rocket& rocket, const Environment& environment, const SimulationOptions& options = {});

}  // namespace rocketup
