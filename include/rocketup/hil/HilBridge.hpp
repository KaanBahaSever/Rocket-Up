#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rocketup/hil/SerialPort.hpp"
#include "rocketup/sim/Simulation.hpp"

namespace rocketup::hil {

/// Text protocol between the simulator and a flight computer (see docs/HIL.md).
///   sim -> device  $RUSEN,t_ms,p_Pa,T_C,ax,ay,az,gx,gy,gz,baroAlt,fix,lat,lon,gpsAlt*CS
///                  $RUEVT,t_ms,event,source*CS          (true events, for logging only)
///   device -> sim  $RUCMD,channel*CS                    (fires Trigger::external(channel))
///                  $RULOG,free text*CS                   (printed by the simulator)
/// CS = two hex digits, XOR of all characters between '$' and '*'. Devices may omit it.
namespace protocol {
std::string checksum(const std::string& body);
std::string frame(const std::string& body);  ///< "$" + body + "*" + checksum + "\n"
std::string sensorLine(const SensorReadings& s);
std::string eventLine(const FlightEvent& e);
/// Parses "$RUCMD,channel*CS" (or "CMD channel"); returns false for other lines.
bool parseCommand(const std::string& line, std::string& channel);
/// Parses "$RULOG,text*CS"; returns false for other lines.
bool parseLog(const std::string& line, std::string& text);
}  // namespace protocol

/// Transport for HIL lines (serial port, in-process emulation, network...).
class Link {
public:
    virtual ~Link() = default;
    virtual void write(const std::string& line) = 0;
    virtual std::vector<std::string> readLines() = 0;
};

class SerialLink final : public Link {
public:
    SerialLink(const std::string& port, int baudRate = 115200);
    void write(const std::string& line) override { port_.write(line); }
    std::vector<std::string> readLines() override { return port_.readLines(); }
    SerialPort& port() { return port_; }

private:
    SerialPort port_;
};

/// Emulated flight computer running inside the simulator process: `device` receives each
/// line sent by the simulator and returns the lines the "device" answers. Lets you test
/// flight software logic (and the protocol) without hardware.
class LoopbackLink final : public Link {
public:
    using Device = std::function<std::vector<std::string>(const std::string& lineFromSimulator)>;
    explicit LoopbackLink(Device device) : device_(std::move(device)) {}
    void write(const std::string& line) override;
    std::vector<std::string> readLines() override;

private:
    Device device_;
    std::vector<std::string> pending_;
};

struct HilOptions {
    double sendRate = 50.0;      ///< Hz of $RUSEN lines
    bool realTime = true;        ///< pace the simulation with the wall clock
    double speedFactor = 1.0;    ///< > 1 runs faster than real time
    double startupDelay = 2.0;   ///< s to wait after opening (Arduino resets on connect)
    bool echoDeviceLogs = true;  ///< print $RULOG lines to stdout
    std::string body;            ///< body carrying the flight computer ("" = launch vehicle)
};

/// Simulation observer that streams simulated sensor data to a flight computer and turns
/// its commands into external trigger channels.
class HilBridge final : public SimulationObserver {
public:
    HilBridge(std::shared_ptr<Link> link, HilOptions options = {});

    void onStart(const Simulation& sim) override;
    void onStep(const std::string& body, const FlightRecord& record, const SensorReadings& sensors) override;
    void onEvent(const FlightEvent& e) override;
    void onFinish(const SimulationResult& result) override;
    void poll(double time, ExternalCommands& commands) override;

    /// Commands received (time, channel), for reports and tests.
    const std::vector<std::pair<double, std::string>>& receivedCommands() const { return received_; }
    const std::vector<std::string>& deviceLog() const { return log_; }

private:
    std::shared_ptr<Link> link_;
    HilOptions opt_;
    std::string body_;
    double nextSend_ = 0.0;
    double lastTime_ = 0.0;
    bool active_ = true;
    std::chrono::steady_clock::time_point wallStart_;
    std::vector<std::pair<double, std::string>> received_;
    std::vector<std::string> log_;
};

}  // namespace rocketup::hil
