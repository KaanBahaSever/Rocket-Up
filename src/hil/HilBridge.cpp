#include "rocketup/hil/HilBridge.hpp"

#include <cstdio>
#include <iostream>
#include <thread>

#include "rocketup/core/Io.hpp"

namespace rocketup::hil {

namespace protocol {

std::string checksum(const std::string& body) {
    unsigned char c = 0;
    for (char ch : body) c ^= static_cast<unsigned char>(ch);
    char buf[3];
    std::snprintf(buf, sizeof buf, "%02X", c);
    return buf;
}

std::string frame(const std::string& body) { return "$" + body + "*" + checksum(body) + "\n"; }

std::string sensorLine(const SensorReadings& s) {
    char buf[320];
    std::snprintf(buf, sizeof buf, "RUSEN,%ld,%.1f,%.2f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%d,%.7f,%.7f,%.1f",
                  static_cast<long>(s.time * 1000.0 + 0.5), s.pressure, s.temperature - 273.15, s.acceleration.x,
                  s.acceleration.y, s.acceleration.z, s.angularRate.x, s.angularRate.y, s.angularRate.z,
                  s.baroAltitude, s.gpsFix ? 1 : 0, s.gpsLatitude, s.gpsLongitude, s.gpsAltitude);
    return frame(buf);
}

std::string eventLine(const FlightEvent& e) {
    return frame("RUEVT," + std::to_string(static_cast<long>(e.time * 1000.0 + 0.5)) + "," + toString(e.type) + "," +
                 e.source);
}

namespace {
/// Strip "$", "*CS" and validate the checksum when present.
bool unframe(const std::string& raw, std::string& body) {
    std::string l = io::trim(raw);
    if (l.empty() || l[0] != '$') return false;
    l.erase(0, 1);
    const size_t star = l.rfind('*');
    if (star != std::string::npos) {
        const std::string cs = l.substr(star + 1);
        l = l.substr(0, star);
        if (!cs.empty() && io::toLower(cs) != io::toLower(checksum(l))) return false;
    }
    body = l;
    return true;
}
}  // namespace

bool parseCommand(const std::string& line, std::string& channel) {
    std::string body;
    if (unframe(line, body)) {
        if (body.rfind("RUCMD,", 0) != 0) return false;
        channel = io::trim(body.substr(6));
        return !channel.empty();
    }
    const auto tok = io::splitWhitespace(line);
    if (tok.size() == 2 && (tok[0] == "CMD" || tok[0] == "DEPLOY" || tok[0] == "FIRE")) {
        channel = tok[1];
        return true;
    }
    return false;
}

bool parseLog(const std::string& line, std::string& text) {
    std::string body;
    if (!unframe(line, body) || body.rfind("RULOG,", 0) != 0) return false;
    text = body.substr(6);
    return true;
}

}  // namespace protocol

SerialLink::SerialLink(const std::string& port, int baudRate) { port_.open(port, baudRate); }

void LoopbackLink::write(const std::string& line) {
    std::string l = line;
    while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
    for (auto& r : device_(l)) pending_.push_back(r);
}

std::vector<std::string> LoopbackLink::readLines() {
    std::vector<std::string> out;
    out.swap(pending_);
    return out;
}

HilBridge::HilBridge(std::shared_ptr<Link> link, HilOptions options) : link_(std::move(link)), opt_(std::move(options)) {
    if (!link_) throw RocketUpError("HIL bridge needs a link");
}

void HilBridge::onStart(const Simulation& sim) {
    body_ = opt_.body.empty() ? sim.rocket().name() : opt_.body;
    if (opt_.startupDelay > 0.0 && dynamic_cast<SerialLink*>(link_.get()))
        std::this_thread::sleep_for(std::chrono::duration<double>(opt_.startupDelay));
    link_->write(protocol::frame("RUHELLO," + sim.rocket().name()));
    nextSend_ = 0.0;
    wallStart_ = std::chrono::steady_clock::now();
}

void HilBridge::onStep(const std::string& body, const FlightRecord& record, const SensorReadings& sensors) {
    if (!active_ || body != body_) return;
    lastTime_ = record.time;
    if (opt_.realTime) {
        const auto target = wallStart_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(record.time / opt_.speedFactor));
        std::this_thread::sleep_until(target);
    }
    if (sensors.valid && record.time + 1e-9 >= nextSend_) {
        nextSend_ = record.time + 1.0 / opt_.sendRate;
        link_->write(protocol::sensorLine(sensors));
    }
    if (record.phase == FlightPhase::Landed) active_ = false;
}

void HilBridge::onEvent(const FlightEvent& e) {
    if (active_ && e.body == body_) link_->write(protocol::eventLine(e));
}

void HilBridge::poll(double time, ExternalCommands& commands) {
    if (!active_) return;
    for (const auto& line : link_->readLines()) {
        std::string ch, text;
        if (protocol::parseCommand(line, ch)) {
            if (!commands.has(ch)) {
                commands.fire(ch);
                received_.emplace_back(time, ch);
                if (opt_.echoDeviceLogs) std::cout << "[HIL] t=" << time << " s  command '" << ch << "'\n";
            }
        } else if (protocol::parseLog(line, text)) {
            log_.push_back(text);
            if (opt_.echoDeviceLogs) std::cout << "[device] " << text << "\n";
        } else if (!line.empty()) {
            log_.push_back(line);
            if (opt_.echoDeviceLogs) std::cout << "[device] " << line << "\n";
        }
    }
}

void HilBridge::onFinish(const SimulationResult&) { link_->write(protocol::frame("RUEND")); }

}  // namespace rocketup::hil
