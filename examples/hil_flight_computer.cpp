// Hardware-in-the-loop: test flight-computer software against the simulator.
//
// The simulator streams noisy sensor data ($RUSEN lines) to a flight computer and the
// computer answers with commands ($RUCMD,drogue / $RUCMD,main) that fire the parachutes.
//
//   hil_flight_computer            -> runs an emulated flight computer in-process (no hardware)
//   hil_flight_computer COM3       -> talks to a real board (e.g. examples/arduino/RocketUpHIL)
//   hil_flight_computer /dev/ttyACM0 115200
//
// The emulated computer below implements the same algorithm as the Arduino sketch, so you
// can develop the logic here first and then port it to the microcontroller.

#include <iostream>
#include <rocketup/RocketUp.hpp>
#include <sstream>

#include "designs/OrbitChaser.hpp"

using namespace rocketup;

namespace {

/// A tiny flight computer: launch detection, barometric apogee, main at 600 m.
class EmulatedFlightComputer {
public:
    std::vector<std::string> onLine(const std::string& line) {
        std::vector<std::string> out;
        if (line.rfind("$RUSEN,", 0) != 0) return out;
        std::vector<double> f;
        std::stringstream ss(line.substr(7, line.find('*') - 7));
        std::string tok;
        while (std::getline(ss, tok, ',')) f.push_back(std::atof(tok.c_str()));
        if (f.size() < 10) return out;
        const double t = f[0] / 1000.0, ax = f[3], alt = f[9];
        if (!launched_ && ax > 3.0 * 9.81) {
            launched_ = true;
            launchTime_ = t;
            out.push_back(hil::protocol::frame("RULOG,launch detected"));
        }
        if (!launched_) return out;
        // Simple exponential filter against baro noise.
        filtered_ = filtered_ < -1e8 ? alt : filtered_ + 0.3 * (alt - filtered_);
        maxAlt_ = std::max(maxAlt_, filtered_);
        if (!drogue_ && t - launchTime_ > 5.0 && maxAlt_ - filtered_ > 4.0) {
            drogue_ = true;
            out.push_back(hil::protocol::frame("RUCMD,drogue"));
            out.push_back(hil::protocol::frame("RULOG,apogee " + std::to_string(static_cast<int>(maxAlt_)) + " m"));
        }
        if (drogue_ && !main_ && filtered_ < 600.0) {
            main_ = true;
            out.push_back(hil::protocol::frame("RUCMD,main"));
        }
        return out;
    }

private:
    bool launched_ = false, drogue_ = false, main_ = false;
    double launchTime_ = 0.0, filtered_ = -1e9, maxAlt_ = -1e9;
};

}  // namespace

int main(int argc, char** argv) {
    const std::string data = ROCKETUP_DATA_DIR;
    try {
        Motor motor = MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor");
        Rocket rocket = designs::buildOrbitChaser(motor);
        // Parachutes now listen to the flight computer instead of builtin logic.
        rocket.findAs<Parachute>("Drogue")->setDeployTrigger(Trigger::external("drogue"));
        rocket.findAs<Parachute>("Main")->setDeployTrigger(Trigger::external("main"));

        Environment env(planets::earth(), LaunchSite{"Aksaray", 38.4, 34.0, 970.0}, LaunchRail{6.0, 85.0, 0.0},
                        std::make_unique<ConstantWind>(4.0, 250.0));

        std::shared_ptr<hil::Link> link;
        hil::HilOptions ho;
        if (argc > 1) {
            link = std::make_shared<hil::SerialLink>(argv[1], argc > 2 ? std::atoi(argv[2]) : 115200);
            ho.realTime = true;
            std::cout << "Connected to " << argv[1] << ", running in real time...\n";
        } else {
            auto fc = std::make_shared<EmulatedFlightComputer>();
            link = std::make_shared<hil::LoopbackLink>([fc](const std::string& l) { return fc->onLine(l); });
            ho.realTime = false;  // emulation can run as fast as possible
            std::cout << "No port given: using the emulated flight computer.\n";
        }
        auto bridge = std::make_shared<hil::HilBridge>(link, ho);
        Simulation sim(rocket, env);
        sim.addObserver(bridge);
        const SimulationResult r = sim.run();
        io::printSummary(std::cout, r);
        std::cout << "\nCommands from the flight computer:\n";
        for (const auto& [t, ch] : bridge->receivedCommands()) std::cout << "  t = " << t << " s: " << ch << "\n";
        const double trueApogee = r.summary.apogeeTime;
        std::cout << "True apogee at t = " << trueApogee << " s\n";
        io::exportAll(r, rocket, env, "output/hil");
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
