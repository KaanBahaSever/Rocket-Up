// Two-stage rocket built only from generic pieces: a separation event splits the stack, a
// trigger ignites the sustainer, and each stage recovers with its own parachute. Every body
// is simulated until landing, so the booster's impact point is known too.

#include <iostream>
#include <rocketup/RocketUp.hpp>

using namespace rocketup;

int main() {
    const std::string data = ROCKETUP_DATA_DIR;
    try {
        const Motor booster = MotorLoader::load(data + "/motors/Cesaroni_7455M2150-P.rumotor");
        const Motor sustainer = MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor");
        const double R = 0.0525;  // 105 mm airframe

        Rocket r("Orbit Chaser II (two stage)");
        // ---------------- sustainer (upper stage)
        auto& nose = r.add<NoseCone>("Nose", NoseShape::Haack, 0.55, R, 0.0);
        nose.setThickness(0.003).setMaterial(materials::fiberglass());
        nose.add<MassObject>("Avionics", 0.6, 0.15, 0.04, "avionics").setPosition(Anchor::Top, 0.25);
        auto& upper = r.add<BodyTube>("Sustainer body", 1.6, R, 0.0025);
        upper.add<Parachute>("Sustainer chute", 1.5, 0.8).setDeployTrigger(Trigger::apogee(1.0)).setPosition(Anchor::Top, 0.05);
        auto& m2 = upper.add<MotorMount>("Sustainer motor", 0.95, 0.0395, 0.002);
        m2.setPosition(Anchor::Bottom, 0.0);
        m2.setMotor(sustainer);
        m2.setIgnitionTrigger(Trigger::afterEvent(EventType::Separation, 1.0, "Staging"));
        upper.addChild(std::make_unique<FinSet>(FinSet::trapezoidal("Sustainer fins", 3, 0.20, 0.08, 0.10, 0.10, 0.004)))
            .setPosition(Anchor::Bottom, 0.0);
        // ---------------- booster
        auto& inter = r.add<BodyTube>("Interstage", 0.2, R, 0.0025);
        inter.overrideMass(0.4);
        auto& lower = r.add<BodyTube>("Booster body", 1.0, R, 0.0025);
        lower.add<Parachute>("Booster chute", 1.0, 0.8).setDeployTrigger(Trigger::apogee(1.0)).setPosition(Anchor::Top, 0.0);
        auto& m1 = lower.add<MotorMount>("Booster motor", 0.95, 0.0395, 0.002);
        m1.setPosition(Anchor::Bottom, 0.0);
        m1.setMotor(booster);
        lower.addChild(std::make_unique<FinSet>(FinSet::trapezoidal("Booster fins", 4, 0.28, 0.12, 0.14, 0.14, 0.005)))
            .setPosition(Anchor::Bottom, 0.0);

        Separation staging;
        staging.name = "Staging";
        staging.aftSectionStart = "Interstage";
        staging.trigger = Trigger::burnout(0.5, "Booster motor");
        staging.forwardName = "Sustainer";
        staging.aftName = "Booster";
        staging.relativeSpeed = 2.0;
        r.addSeparation(staging);

        Environment env(planets::earth(), LaunchSite{"Desert", 35.0, 33.0, 300.0}, LaunchRail{8.0, 87.0, 90.0},
                        std::make_unique<ConstantWind>(3.0, 300.0));
        SimulationOptions o;
        o.maxTime = 1500.0;
        io::printRocketSummary(std::cout, r);
        const SimulationResult res = simulate(r, env, o);
        io::printSummary(std::cout, res);
        io::exportAll(res, r, env, "output/two_stage");
        r.save("output/two_stage/two_stage.rocketup");
        std::cout << "\nReport: output/two_stage/report.html\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
