// Same rocket, different worlds.
//
// Planets are data: builtin Earth/Mars/Moon plus any JSON planet file (data/planets/*.json:
// Venus, Titan, ... or your own). Gravity, atmosphere (temperature, pressure, density, speed of
// sound, viscosity -> Reynolds number -> skin friction) and the motor's pressure thrust all
// change with the world.

#include <iomanip>
#include <iostream>
#include <rocketup/RocketUp.hpp>

#include "designs/OrbitChaser.hpp"

using namespace rocketup;

int main() {
    const std::string data = ROCKETUP_DATA_DIR;
    try {
        Motor motor = MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor");
        // A known nozzle exit lets the thrust follow the ambient pressure: F = F_test + (p_test - p_amb) A_e.
        motor.nozzleExitArea = mathrix::kPi * 0.045 * 0.045 / 4.0;
        motor.finalize();
        const Rocket rocket = designs::buildOrbitChaser(motor);

        std::vector<Planet> worlds = {planets::earth(), planets::mars(), planets::moon(),
                                      Planet::loadFile(data + "/planets/titan.json"),
                                      Planet::loadFile(data + "/planets/venus.json")};
        std::cout << "world        g0 (m/s2)  rho0 (kg/m3)   apogee (m)  vmax (m/s)  Mach   t_apogee (s)  flight (s)\n";
        for (auto& w : worlds) {
            const std::string name = w.name();
            const double g0 = w.surfaceGravity();
            const double rho0 = w.atmosphere().at(0.0).density;
            Environment env(std::move(w), LaunchSite{"pad", 0.0, 0.0, 0.0}, LaunchRail{6.0, 85.0, 0.0});
            SimulationOptions o;
            o.maxTime = 7200.0;
            const SimulationResult r = simulate(rocket, env, o);
            const auto& s = r.summary;
            std::cout << std::left << std::setw(12) << name << std::right << std::fixed << std::setprecision(3)
                      << std::setw(10) << g0 << std::setprecision(4) << std::setw(14) << rho0 << std::setprecision(0)
                      << std::setw(13) << s.apogee << std::setw(12) << s.maxSpeed << std::setprecision(2) << std::setw(7)
                      << s.maxMach << std::setprecision(1) << std::setw(14) << s.apogeeTime << std::setw(12)
                      << s.flightTime << "\n";
            if (name == "Mars") io::exportAll(r, rocket, env, "output/mars");
        }
        std::cout << "\nMars report: output/mars/report.html\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
