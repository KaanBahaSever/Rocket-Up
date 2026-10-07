// Open a saved project (rocket + environment + options), attach your own trigger logic
// by name, tweak it and run. This is the workflow a GUI or a batch script would use.
//
//   load_design [project.json]

#include <iostream>
#include <rocketup/RocketUp.hpp>

using namespace rocketup;

int main(int argc, char** argv) {
    const std::string data = ROCKETUP_DATA_DIR;
    const std::string path = argc > 1 ? argv[1] : data + "/projects/orbit_chaser.project.json";
    try {
        // Custom trigger functions are referenced by name in design files. Register the C++
        // implementation before loading; otherwise the stored builtin fallback is used.
        Trigger::registerNamed("baro apogee (5 m drop, mach < 0.3)", [](const TriggerContext& c) {
            return c.sensors.valid && c.state.mach < 0.3 && c.sensors.maxBaroAltitude > 100.0 &&
                   c.sensors.maxBaroAltitude - c.sensors.baroAltitude > 5.0;
        });

        Project p = Project::load(path);
        std::cout << "Loaded '" << p.name << "' (" << p.rocket.stack().size() << " stack components, planet "
                  << p.environment.planet().name() << ")\n";

        // Edit anything before running, e.g. a bigger main parachute and a steeper rail.
        if (auto* main = p.rocket.findAs<Parachute>("Main")) main->setDiameter(2.4);
        LaunchRail rail = p.environment.rail();
        rail.elevation = 88.0;
        p.environment.setRail(rail);

        const SimulationResult r = Simulation(p.rocket, p.environment, p.options).run();
        io::printSummary(std::cout, r);
        io::exportAll(r, p.rocket, p.environment, "output/load_design");
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
