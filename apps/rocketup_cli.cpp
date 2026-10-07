// rocketup - command line front-end of the Rocket-Up library.
//
//   rocketup run <project.json | design.rocketup> [-o DIR] [--integrator rk4] [--dt 0.01] [--dynamics 6dof]
//   rocketup info <design.rocketup | project.json> [--svg FILE]
//   rocketup motor <file> [--isp S] [--propellant KG] [--total KG] [--convert OUT.rumotor]
//   rocketup planet <earth|mars|moon|file.json> [--export FILE] [--table MAX_ALT]
//   rocketup montecarlo <project.json> [--runs N] [-o DIR]
//   rocketup hil <project.json> --port COM3 [--baud 115200] [--rate 50] [--speed 1]
//   rocketup ports

#include <rocketup/RocketUp.hpp>

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>

using namespace rocketup;

namespace {

struct Args {
    std::vector<std::string> positional;
    std::map<std::string, std::string> options;
    bool has(const std::string& k) const { return options.count(k) > 0; }
    std::string get(const std::string& k, const std::string& def = "") const {
        auto it = options.find(k);
        return it == options.end() ? def : it->second;
    }
    double num(const std::string& k, double def) const { return has(k) ? std::stod(get(k)) : def; }
};

Args parse(int argc, char** argv, int start) {
    Args a;
    for (int i = start; i < argc; ++i) {
        std::string s = argv[i];
        if (s.rfind("--", 0) == 0 || (s.size() == 2 && s[0] == '-')) {
            std::string key = s.substr(s[1] == '-' ? 2 : 1);
            if (key == "o") key = "out";
            if (i + 1 < argc && argv[i + 1][0] != '-') a.options[key] = argv[++i];
            else a.options[key] = "1";
        } else {
            a.positional.push_back(s);
        }
    }
    return a;
}

void usage() {
    std::cout << "Rocket-Up " << ROCKETUP_VERSION_STRING << " - modular rocket flight simulator\n\n"
              << "usage:\n"
              << "  rocketup run <project.json|design.rocketup> [-o DIR] [--integrator euler|midpoint|heun|rk4|rk45]\n"
              << "               [--dt SECONDS] [--dynamics 6dof|3dof] [--apogee-only] [--quiet]\n"
              << "  rocketup info <design.rocketup|project.json> [--svg FILE]\n"
              << "  rocketup motor <file.eng|.rse|.csv|.rumotor> [--isp S] [--propellant KG] [--total KG]\n"
              << "               [--interp linear|spline|akima|pchip] [--convert OUT.rumotor]\n"
              << "  rocketup planet <earth|mars|moon|planet.json> [--export FILE] [--table MAX_ALT]\n"
              << "  rocketup montecarlo <project.json> [--runs N] [--threads N] [-o DIR]\n"
              << "  rocketup hil <project.json> --port PORT [--baud 115200] [--rate 50] [--speed 1]\n"
              << "  rocketup ports\n";
}

void applySimArgs(SimulationOptions& o, const Args& a) {
    if (a.has("integrator")) o.integrator = mathrix::integratorFromString(a.get("integrator"));
    if (a.has("dt")) o.timeStep = a.num("dt", o.timeStep);
    if (a.has("dynamics")) o.dynamics = dynamicsModelFromString(a.get("dynamics"));
    if (a.has("apogee-only")) o.stopAtApogee = true;
    if (a.has("max-time")) o.maxTime = a.num("max-time", o.maxTime);
}

int cmdRun(const Args& a) {
    if (a.positional.empty()) throw RocketUpError("run: missing project file");
    Project p = Project::loadAny(a.positional[0]);
    applySimArgs(p.options, a);
    const std::string out = a.get("out", p.outputDirectory);
    if (!a.has("quiet")) io::printRocketSummary(std::cout, p.rocket);
    Simulation sim(p.rocket, p.environment, p.options);
    const SimulationResult r = sim.run();
    io::printSummary(std::cout, r);
    const auto files = io::exportAll(r, p.rocket, p.environment, out);
    std::cout << "\nWrote " << files.size() << " files to " << out << ":\n";
    for (const auto& f : files) std::cout << "  " << f << "\n";
    return 0;
}

int cmdInfo(const Args& a) {
    if (a.positional.empty()) throw RocketUpError("info: missing design file");
    Project p = Project::loadAny(a.positional[0]);
    io::printRocketSummary(std::cout, p.rocket);
    AeroModel aero(p.rocket);
    const MassProperties mp = p.rocket.massProperties(-1.0);
    std::cout << "\n  Component aerodynamics at Mach 0.3:\n";
    FlightConditions c;
    c.mach = 0.3;
    c.airspeed = 102.0;
    std::cout << std::fixed << std::setprecision(4);
    for (const auto& ca : aero.breakdown(c))
        std::cout << "    " << std::left << std::setw(26) << ca.name << std::right << " CNa " << std::setw(7) << ca.cnAlpha
                  << "  CP " << std::setw(7) << ca.cp << "  Cd f/p/b " << ca.cdFriction << " / " << ca.cdPressure
                  << " / " << ca.cdBase << "\n";
    std::cout << "\n  Component tree:\n";
    for (const auto& pc : p.rocket.flatten()) {
        int depth = 0;
        for (const Component* q = pc.component->parent(); q; q = q->parent()) ++depth;
        const MassProperties m = pc.component->mass();
        std::cout << "    " << std::string(static_cast<size_t>(depth) * 2, ' ') << pc.component->name() << " ["
                  << pc.component->type() << "]  x=" << std::setprecision(3) << pc.position << " m, " << m.mass << " kg\n";
    }
    (void)mp;
    if (a.has("svg")) {
        io::writeTextFile(a.get("svg"), io::drawRocketSvg(p.rocket));
        std::cout << "\nDrawing written to " << a.get("svg") << "\n";
    }
    return 0;
}

int cmdMotor(const Args& a) {
    if (a.positional.empty()) throw RocketUpError("motor: missing motor file");
    MotorLoadOptions o;
    if (a.has("isp")) o.isp = a.num("isp", 0);
    if (a.has("propellant")) o.propellantMass = a.num("propellant", 0);
    if (a.has("total")) o.totalMass = a.num("total", 0);
    if (a.has("interp")) o.interpolation = mathrix::interpFromString(a.get("interp"));
    for (const Motor& m : MotorLoader::loadAll(a.positional[0], o)) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << m.displayName() << " (" << m.impulseClass() << ", " << m.motorType << ")\n"
                  << "  diameter " << m.diameter * 1000.0 << " mm, length " << m.length * 1000.0 << " mm\n"
                  << "  total impulse " << m.totalImpulse() << " Ns, burn time " << m.burnTime() << " s\n"
                  << "  average thrust " << m.averageThrust() << " N, max thrust " << m.maxThrust() << " N\n"
                  << "  propellant " << m.propellantMass << " kg, loaded " << m.totalMass << " kg, Isp "
                  << m.effectiveIsp() << " s\n"
                  << "  interpolation " << mathrix::toString(m.interpolation) << ", mass model " << toString(m.massModel)
                  << "\n";
        for (const auto& w : m.warnings()) std::cout << "  ! " << w << "\n";
        if (a.has("convert")) {
            MotorLoader::save(m, a.get("convert"));
            std::cout << "  saved " << a.get("convert") << "\n";
        }
    }
    return 0;
}

int cmdPlanet(const Args& a) {
    if (a.positional.empty()) throw RocketUpError("planet: missing name or file");
    const std::string arg = a.positional[0];
    Planet p = io::extensionOf(arg) == ".json" ? Planet::loadFile(arg) : planets::byName(arg);
    std::cout << p.name() << ": GM " << p.gravitationalParameter() << " m^3/s^2, radius " << p.radius() / 1000.0
              << " km, g0 " << p.surfaceGravity() << " m/s^2, atmosphere " << p.atmosphere().type() << "\n";
    if (a.has("table")) {
        const double top = a.num("table", 20000.0);
        std::cout << "  altitude_m  temperature_K  pressure_Pa  density_kgm3  speed_of_sound_mps  gravity\n";
        for (int i = 0; i <= 20; ++i) {
            const double h = top * i / 20.0;
            const AtmosphereState s = p.atmosphere().at(h);
            std::cout << std::fixed << std::setprecision(1) << std::setw(11) << h << std::setw(15) << s.temperature
                      << std::setw(13) << s.pressure << std::setprecision(5) << std::setw(14) << s.density
                      << std::setprecision(2) << std::setw(20) << s.speedOfSound << std::setprecision(4) << std::setw(9)
                      << p.gravity(h) << "\n";
        }
    }
    if (a.has("export")) {
        p.saveFile(a.get("export"));
        std::cout << "  saved " << a.get("export") << "\n";
    }
    return 0;
}

int cmdMonteCarlo(const Args& a) {
    if (a.positional.empty()) throw RocketUpError("montecarlo: missing project file");
    Project p = Project::loadAny(a.positional[0]);
    MonteCarloOptions o;
    o.simulation = p.options;
    applySimArgs(o.simulation, a);
    o.runs = static_cast<int>(a.num("runs", 200));
    o.threads = static_cast<int>(a.num("threads", 0));
    const std::string out = a.get("out", p.outputDirectory + "/montecarlo");
    const MonteCarloResult r = runMonteCarlo(p.rocket, p.environment, o, [](int d, int n) {
        if (d % std::max(1, n / 20) == 0 || d == n) std::cout << "\r  " << d << " / " << n << " runs" << std::flush;
    });
    std::cout << "\n\nApogee " << std::fixed << std::setprecision(1) << r.apogeeMean << " +- " << r.apogeeStd << " m (min "
              << r.apogeeMin << ", max " << r.apogeeMax << ")\n";
    for (const auto& l : r.landings)
        std::cout << "  " << l.body << ": mean landing (" << l.mean.x << ", " << l.mean.y << ") m, 1-sigma ellipse "
                  << l.semiMajor << " x " << l.semiMinor << " m, max distance " << l.maxDistance << " m\n";
    std::filesystem::create_directories(std::filesystem::u8path(out));
    io::writeJsonFile(out + "/montecarlo.json", r.toJson());
    writeMonteCarloReport(r, p.rocket, p.environment, out + "/montecarlo.html");
    writeMonteCarloKml(r, p.environment, out + "/montecarlo.kml");
    std::cout << "Wrote " << out << "/montecarlo.{json,html,kml} (" << r.computeTime << " s)\n";
    return 0;
}

int cmdHil(const Args& a) {
    if (a.positional.empty() || !a.has("port")) throw RocketUpError("hil: usage rocketup hil <project> --port PORT");
    Project p = Project::loadAny(a.positional[0]);
    applySimArgs(p.options, a);
    hil::HilOptions ho;
    ho.sendRate = a.num("rate", 50.0);
    ho.speedFactor = a.num("speed", 1.0);
    auto link = std::make_shared<hil::SerialLink>(a.get("port"), static_cast<int>(a.num("baud", 115200)));
    auto bridge = std::make_shared<hil::HilBridge>(link, ho);
    Simulation sim(p.rocket, p.environment, p.options);
    sim.addObserver(bridge);
    std::cout << "HIL running on " << a.get("port") << " (Ctrl+C to abort)...\n";
    const SimulationResult r = sim.run();
    io::printSummary(std::cout, r);
    io::exportAll(r, p.rocket, p.environment, a.get("out", p.outputDirectory + "/hil"));
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    const std::string cmd = argv[1];
    try {
        const Args a = parse(argc, argv, 2);
        if (cmd == "run") return cmdRun(a);
        if (cmd == "info") return cmdInfo(a);
        if (cmd == "motor") return cmdMotor(a);
        if (cmd == "planet") return cmdPlanet(a);
        if (cmd == "montecarlo" || cmd == "mc") return cmdMonteCarlo(a);
        if (cmd == "hil") return cmdHil(a);
        if (cmd == "ports") {
            for (const auto& p : hil::SerialPort::listPorts()) std::cout << p << "\n";
            return 0;
        }
        if (cmd == "version" || cmd == "--version") {
            std::cout << "Rocket-Up " << ROCKETUP_VERSION_STRING << "\n";
            return 0;
        }
        usage();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }
}
