// Orbit Chaser - complete example.
//
// Builds a rocket in code, defines the launch environment, attaches parachute and payload
// triggers (builtin and custom C++ lambdas), runs a 6-DOF simulation, prints the results and
// exports CSV / JSON / KML / an interactive HTML report. Finally the design and the whole
// project are saved so they can be re-opened later (or edited by hand / a GUI).

#include <iostream>
#include <rocketup/RocketUp.hpp>

#include "designs/OrbitChaser.hpp"

using namespace rocketup;

int main(int argc, char** argv) {
    const std::string data = ROCKETUP_DATA_DIR;
    const std::string out = argc > 1 ? argv[1] : "output/orbit_chaser";
    try {
        // ---------------------------------------------------------------- motor
        // Motors are data, not code: any .eng / .rse / ThrustCurve .csv / .rumotor file works.
        Motor motor = MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor");
        for (const auto& w : motor.warnings()) std::cout << "motor: " << w << "\n";

        // ---------------------------------------------------------------- rocket
        Rocket rocket = designs::buildOrbitChaser(motor);

        // ---------------------------------------------------------------- environment
        // TEKNOFEST launch area near Aksaray (Turkey), measured launch-site weather and wind.
        Planet earth = planets::earth();
        Environment env(std::move(earth), LaunchSite{"Aksaray / Tuz Golu", 38.4, 34.0, 970.0},
                        LaunchRail{6.0, 85.0, 0.0},
                        std::make_unique<TurbulentWind>(std::make_unique<PowerLawWind>(5.0, 270.0, 10.0), 0.1, 7));
        if (auto* atm = dynamic_cast<LayeredAtmosphere*>(&env.planet().atmosphere()))
            atm->setConditionsAt(970.0, 273.15 + 24.0, 90200.0);  // 24 C, 902 hPa at the pad

        // ---------------------------------------------------------------- simulation
        SimulationOptions options;
        options.integrator = mathrix::Integrator::RK4;
        options.dynamics = DynamicsModel::SixDof;
        options.timeStep = 0.01;

        Simulation sim(rocket, env, options);
        const SimulationResult result = sim.run();

        io::printRocketSummary(std::cout, rocket);
        io::printSummary(std::cout, result);

        // ---------------------------------------------------------------- export
        const auto files = io::exportAll(result, rocket, env, out);
        rocket.save(out + "/orbit_chaser.rocketup");
        Project project{"Orbit Chaser at TEKNOFEST", rocket, env, options, out, 0.0};
        project.save(out + "/orbit_chaser.project.json");
        std::cout << "\nFiles:\n";
        for (const auto& f : files) std::cout << "  " << f << "\n";
        std::cout << "  " << out << "/orbit_chaser.rocketup\n  " << out << "/orbit_chaser.project.json\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
