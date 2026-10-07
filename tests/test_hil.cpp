#include <nlohmann/json.hpp>
#include <rocketup/RocketUp.hpp>

#include "../examples/designs/OrbitChaser.hpp"
#include "TestFramework.hpp"

using namespace rocketup;

TEST_CASE("hil: protocol framing and parsing") {
    const std::string f = hil::protocol::frame("RUCMD,drogue");
    CHECK(f.front() == '$' && f.back() == '\n');
    std::string ch;
    CHECK(hil::protocol::parseCommand(f, ch) && ch == "drogue");
    CHECK(hil::protocol::parseCommand("$RUCMD,main", ch) && ch == "main");  // checksum optional
    CHECK(!hil::protocol::parseCommand("$RUCMD,main*00", ch));             // wrong checksum rejected
    CHECK(hil::protocol::parseCommand("DEPLOY drogue", ch) && ch == "drogue");
    std::string text;
    CHECK(hil::protocol::parseLog(hil::protocol::frame("RULOG,hello world"), text) && text == "hello world");
    SensorReadings s;
    s.time = 1.234;
    s.pressure = 90000.0;
    s.baroAltitude = 12.5;
    const std::string line = hil::protocol::sensorLine(s);
    CHECK(line.rfind("$RUSEN,1234,90000.0,", 0) == 0);
    // Checksum of the body between '$' and '*'.
    const std::string body = line.substr(1, line.find('*') - 1);
    CHECK(line.substr(line.find('*') + 1, 2) == hil::protocol::checksum(body));
}

TEST_CASE("hil: emulated flight computer deploys the parachutes") {
    const std::string data = ROCKETUP_DATA_DIR;
    Rocket r = designs::buildOrbitChaser(MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor"));
    r.findAs<Parachute>("Drogue")->setDeployTrigger(Trigger::external("drogue"));
    r.findAs<Parachute>("Main")->setDeployTrigger(Trigger::external("main"));
    // Flight computer: drogue when the baro altitude field dropped 10 m below its maximum,
    // main below 500 m afterwards.
    double maxAlt = -1e9;
    bool drogue = false, mainSent = false;
    auto device = [&](const std::string& l) -> std::vector<std::string> {
        if (l.rfind("$RUSEN,", 0) != 0) return {};
        std::vector<double> v;
        size_t p = 7;
        while (p < l.size() && l[p] != '*') {
            const size_t e = l.find_first_of(",*", p);
            v.push_back(std::atof(l.substr(p, e - p).c_str()));
            p = e + (l[e] == ',' ? 1 : 0);
            if (l[e] == '*') break;
        }
        const double alt = v.at(9);
        maxAlt = std::max(maxAlt, alt);
        if (!drogue && maxAlt > 100 && maxAlt - alt > 10) {
            drogue = true;
            return {hil::protocol::frame("RUCMD,drogue")};
        }
        if (drogue && !mainSent && alt < 500) {
            mainSent = true;
            return {hil::protocol::frame("RUCMD,main")};
        }
        return {};
    };
    hil::HilOptions o;
    o.realTime = false;
    o.echoDeviceLogs = false;
    auto bridge = std::make_shared<hil::HilBridge>(std::make_shared<hil::LoopbackLink>(device), o);
    Simulation sim(r, Environment(planets::earth(), LaunchSite{"", 38.4, 34.0, 970.0}));
    sim.addObserver(bridge);
    const SimulationResult res = sim.run();
    CHECK(bridge->receivedCommands().size() == 2);
    const BodyResult& b = res.bodies.front();
    for (const auto& rec : b.recovery) {
        CHECK(rec.deployTime > 0.0);
        if (rec.device == "Drogue") CHECK(rec.deployTime > res.summary.apogeeTime);
        if (rec.device == "Main") CHECK_NEAR(rec.deployAltitude, 500.0, 25.0);
    }
}
