// TEKNOFEST 2021 3-DOF validation data set, reproduced with Rocket-Up.
//
// Competition rules: flat earth, point mass, no wind, constant Isp, Cd from the supplied
// Cd(Mach, altitude) table, US Standard Atmosphere, constant gravity, Euler integration with a
// 10 ms step, simulation until apogee. Inputs: 25 kg initial mass, 4.659 kg propellant,
// Isp 209.5 s, 85 deg launch elevation, supplied thrust profile.
//
// The same rocket is then re-run with RK4 and with the full 6-DOF model so the effect of
// each simplification is visible.

#include <iomanip>
#include <iostream>
#include <rocketup/RocketUp.hpp>

using namespace rocketup;

namespace {

Rocket buildCompetitionRocket(const std::string& data) {
    MotorLoadOptions mo;
    mo.designation = "TEKNOFEST 2021 validation motor";
    mo.propellantMass = 4.659;
    mo.totalMass = 4.659;  // the data set gives only the propellant mass
    mo.isp = 209.5;
    mo.diameter = 0.098;
    mo.length = 0.70;
    mo.massModel = MassFlowModel::ConstantIsp;  // mdot = F / (Isp g0), as required
    mo.interpolation = mathrix::Interp::Linear;
    Motor motor = MotorLoader::load(data + "/motors/source/teknofest2021_validation_thrust.csv", mo);

    // Only the reference area (0.0153 m^2) matters for a table-driven 3-DOF run.
    const double R = std::sqrt(0.0153 / mathrix::kPi);
    Rocket r("TEKNOFEST 2021 validation rocket");
    r.add<NoseCone>("Nose", NoseShape::Ogive, 0.55, R, 1.0);
    auto& body = r.add<BodyTube>("Body", 2.4, R, 0.003);
    body.add<MotorMount>("Motor mount", 0.75, 0.051, 0.002).setPosition(Anchor::Bottom, 0.0);
    static_cast<MotorMount&>(*body.children().back()).setMotor(motor);
    auto& fins = body.addChild(std::make_unique<FinSet>(FinSet::trapezoidal("Fins", 4, 0.3, 0.12, 0.15, 0.15, 0.004)));
    fins.setPosition(Anchor::Bottom, 0.0);
    r.overrideDryMass(25.0 - motor.totalMass, 1.6);
    r.aero().loadTable(data + "/aero/teknofest2021_cd_vs_mach.csv");
    return r;
}

void report(const char* label, const SimulationResult& res) {
    const auto& s = res.summary;
    const FlightRecord& last = res.bodies.front().records.back();
    std::cout << std::left << std::setw(28) << label << std::right << std::fixed << std::setprecision(1)
              << std::setw(9) << s.apogee << std::setw(9) << s.apogeeTime << std::setw(9) << s.maxSpeed << std::setw(8)
              << std::setprecision(3) << s.maxMach << std::setprecision(1) << std::setw(9) << s.maxAcceleration
              << std::setw(9) << s.railExitSpeed << std::setw(10) << std::hypot(last.position.x, last.position.y)
              << std::setprecision(3) << std::setw(9) << s.propellantBurned << "\n";
}

}  // namespace

int main() {
    const std::string data = ROCKETUP_DATA_DIR;
    try {
        const Rocket rocket = buildCompetitionRocket(data);
        Planet earth = planets::earth();
        earth.setConstantGravity(9.804);  // rule 1.7 of the competition
        const Environment env(std::move(earth), LaunchSite{"Sea level", 39.0, 35.0, 0.0}, LaunchRail{6.0, 85.0, 0.0});

        std::cout << "                              apogee    t_apo     vmax    Mmax     amax  railExit     range  propUsed\n";
        std::cout << "                                 (m)      (s)    (m/s)            (m/s2)    (m/s)       (m)      (kg)\n";

        SimulationOptions rules;
        rules.dynamics = DynamicsModel::ThreeDof;
        rules.integrator = mathrix::Integrator::Euler;
        rules.timeStep = 0.01;
        rules.stopAtApogee = true;
        const SimulationResult official = simulate(rocket, env, rules);
        report("3-DOF Euler 10 ms (rules)", official);

        SimulationOptions rk4 = rules;
        rk4.integrator = mathrix::Integrator::RK4;
        report("3-DOF RK4 10 ms", simulate(rocket, env, rk4));

        SimulationOptions six = rk4;
        six.dynamics = DynamicsModel::SixDof;
        report("6-DOF RK4 10 ms", simulate(rocket, env, six));

        std::cout << "\nOld Python implementation (2021): apogee 4385 m, t_apo 29.0 s, Mach 1.06, range 1391 m\n";
        io::exportAll(official, rocket, env, "output/teknofest_3dof");
        std::cout << "Report written to output/teknofest_3dof/report.html\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
