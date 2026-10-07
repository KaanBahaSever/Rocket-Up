// Monte Carlo dispersion: where will the rocket, and each separately recovered payload, land?
// Wind, thrust, drag, mass, rail angle and parachute Cd are perturbed; runs execute in parallel.

#include <iostream>
#include <rocketup/RocketUp.hpp>

#include "designs/OrbitChaser.hpp"

using namespace rocketup;

int main(int argc, char** argv) {
    const std::string data = ROCKETUP_DATA_DIR;
    try {
        const Rocket rocket = designs::buildOrbitChaser(MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor"));
        const Environment env(planets::earth(), LaunchSite{"Aksaray", 38.4, 34.0, 970.0}, LaunchRail{6.0, 85.0, 0.0},
                              std::make_unique<PowerLawWind>(6.0, 270.0));
        MonteCarloOptions o;
        o.runs = argc > 1 ? std::atoi(argv[1]) : 200;
        o.simulation.timeStep = 0.02;
        o.simulation.integrator = mathrix::Integrator::RK4;
        const MonteCarloResult r = runMonteCarlo(rocket, env, o, [](int d, int n) {
            if (d % 20 == 0 || d == n) std::cout << "\r" << d << "/" << n << std::flush;
        });
        std::cout << "\nApogee " << r.apogeeMean << " +- " << r.apogeeStd << " m\n";
        for (const auto& l : r.landings)
            std::cout << "  " << l.body << ": mean (" << l.mean.x << ", " << l.mean.y << ") m, 1-sigma ellipse "
                      << l.semiMajor << " x " << l.semiMinor << " m @ " << l.angle << " deg\n";
        writeMonteCarloReport(r, rocket, env, "output/monte_carlo/montecarlo.html");
        writeMonteCarloKml(r, env, "output/monte_carlo/montecarlo.kml");
        io::writeJsonFile("output/monte_carlo/montecarlo.json", r.toJson());
        std::cout << "Report: output/monte_carlo/montecarlo.html (" << r.computeTime << " s)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
