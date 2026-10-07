#include <nlohmann/json.hpp>
#include <rocketup/RocketUp.hpp>

#include "../examples/designs/OrbitChaser.hpp"
#include "TestFramework.hpp"

using namespace rocketup;

namespace {
const std::string data = ROCKETUP_DATA_DIR;

Environment aksaray(double windFrom = -1.0, double windSpeed = 6.0) {
    return Environment(planets::earth(), LaunchSite{"Aksaray", 38.4, 34.0, 970.0}, LaunchRail{6.0, 85.0, 0.0},
                       windFrom < 0.0 ? nullptr : std::make_unique<ConstantWind>(windSpeed, windFrom));
}
}  // namespace

TEST_CASE("simulation: vacuum flight obeys energy conservation") {
    Planet p("Airless", 4.9e12, 1.7e6, std::make_unique<VacuumAtmosphere>(), 0.0);
    p.setConstantGravity(1.62);
    Environment env(std::move(p), LaunchSite{}, LaunchRail{2.0, 90.0, 0.0});
    Rocket r("Probe");
    r.add<NoseCone>("Nose", NoseShape::Ogive, 0.3, 0.05);
    auto& body = r.add<BodyTube>("Body", 1.0, 0.05, 0.002);
    body.add<MotorMount>("Mount", 0.8, 0.04, 0.002).setPosition(Anchor::Bottom, 0.0);
    static_cast<MotorMount&>(*body.children().back()).setMotor(MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor"));
    body.addChild(std::make_unique<FinSet>(FinSet::trapezoidal("Fins", 4, 0.15, 0.08, 0.1, 0.05, 0.003)))
        .setPosition(Anchor::Bottom, 0.0);
    SimulationOptions o;
    o.stopAtApogee = true;
    const SimulationResult res = simulate(r, env, o);
    const auto& s = res.summary;
    CHECK(s.burnoutTime > 4.0);
    const double predicted = s.burnoutAltitude + s.burnoutSpeed * s.burnoutSpeed / (2.0 * 1.62);
    CHECK_REL(s.apogee, predicted, 2e-3);
    // Burnout speed from the rocket equation with gravity loss: v = Isp g0 ln(m0/m1) - g t_b.
    const Motor& m = r.motorMounts().front()->motor();
    const double m0 = s.liftoffMass, m1 = m0 - m.propellantMass;
    const double ideal = m.effectiveIsp() * constants::kStandardGravity * std::log(m0 / m1) - 1.62 * m.burnTime();
    CHECK_REL(s.burnoutSpeed, ideal, 5e-3);
}

TEST_CASE("simulation: Orbit Chaser matches OpenRocket within 0.5 %") {
    // References: the team's OpenRocket 15.03 models (6 m/s wind from the south, rail tilted
    // 5 deg to the north, 970 m site). OpenRocket does not reduce base drag during the burn.
    struct Case {
        const char* motor;
        double dryMass, dryCg;
        double apogee, vmax, railExit, tApogee, maxMach;
    };
    const Case cases[] = {{"Cesaroni_8429M2020-P", 21.229, 0.886, 3111.6, 263.46, 31.95, 25.39, 0.78791},
                          {"Cesaroni_7455M2150-P", 18.715, 0.8959, 2985.5, 272.08, 32.59, 24.61, 0.81281}};
    for (const auto& c : cases) {
        Rocket r = designs::buildOrbitChaser(MotorLoader::load(data + "/motors/" + c.motor + ".rumotor"));
        r.overrideDryMass(c.dryMass, c.dryCg);
        r.aero().plumeReducesBaseDrag = false;
        const SimulationResult res = simulate(r, aksaray(180.0));
        CHECK_REL(res.summary.apogee, c.apogee, 0.005);
        CHECK_REL(res.summary.maxSpeed, c.vmax, 0.005);
        CHECK_REL(res.summary.railExitSpeed, c.railExit, 0.01);
        CHECK_REL(res.summary.apogeeTime, c.tApogee, 0.005);
        CHECK_REL(res.summary.maxMach, c.maxMach, 0.005);
    }
}

TEST_CASE("simulation: integrators and dynamics models agree") {
    const Rocket r = designs::buildOrbitChaser(MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor"));
    SimulationOptions base;
    base.stopAtApogee = true;
    const double ref = simulate(r, aksaray(), base).summary.apogee;
    for (auto integ : {mathrix::Integrator::Midpoint, mathrix::Integrator::Heun, mathrix::Integrator::DormandPrince}) {
        SimulationOptions o = base;
        o.integrator = integ;
        CHECK_REL(simulate(r, aksaray(), o).summary.apogee, ref, 0.005);
    }
    SimulationOptions o3 = base;
    o3.dynamics = DynamicsModel::ThreeDof;
    CHECK_REL(simulate(r, aksaray(), o3).summary.apogee, ref, 0.01);
    SimulationOptions fine = base;
    fine.timeStep = 0.002;
    CHECK_REL(simulate(r, aksaray(), fine).summary.apogee, ref, 0.002);
}

TEST_CASE("simulation: recovery, payload ejection and landing") {
    const Rocket r = designs::buildOrbitChaser(MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor"));
    const SimulationResult res = simulate(r, aksaray(270.0, 5.0));
    CHECK(res.bodies.size() == 2);
    const BodyResult* rocket = res.body("Orbit Chaser");
    const BodyResult* payload = res.body("Payload");
    CHECK(rocket && payload);
    CHECK(rocket->landing.landed && payload->landing.landed);
    CHECK_NEAR(payload->mass, 4.025 + 0.0758 + 0.0108, 0.05);
    // Main chute at 600 m (Cd 0.8, 2 m): terminal speed sqrt(2 m g / (rho Cd A)).
    const double m = rocket->mass, rho = 1.08, A = mathrix::kPi;
    CHECK_NEAR(rocket->landing.descentRate, std::sqrt(2 * m * 9.8 / (rho * (0.8 * A + 0.6 * 0.26))), 1.5);
    // Drogue fires shortly after apogee (sensor based), main around 600 m.
    for (const auto& rec : rocket->recovery) {
        if (rec.device == "Drogue") CHECK(rec.deployTime > res.summary.apogeeTime && rec.deployTime < res.summary.apogeeTime + 3.0);
        if (rec.device == "Main") CHECK_NEAR(rec.deployAltitude, 600.0, 5.0);
        CHECK(rec.peakForce > 0.0);
    }
    // Wind from the west carries everything east.
    CHECK(rocket->landing.position.x > 0.0 && payload->landing.position.x > rocket->landing.position.x);
    const double apogeeTime = res.summary.apogeeTime;
    const FlightEvent* ej = payload->events.find(EventType::Ejection);
    CHECK(ej && std::abs(ej->time - (apogeeTime + 1.0)) < 0.05);
}

TEST_CASE("simulation: saved designs reproduce the same flight") {
    const Rocket r = designs::buildOrbitChaser(MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor"));
    Trigger::registerNamed("baro apogee (5 m drop, mach < 0.3)", [](const TriggerContext& c) {
        return c.sensors.valid && c.state.mach < 0.3 && c.sensors.maxBaroAltitude > 100.0 &&
               c.sensors.maxBaroAltitude - c.sensors.baroAltitude > 5.0;
    });
    const Rocket q = Rocket::fromJson(r.toJson());
    const Environment env = aksaray(200.0, 4.0);
    const Environment env2 = Environment::fromJson(env.toJson());
    const SimulationResult a = simulate(r, env), b = simulate(q, env2);
    CHECK_NEAR(a.summary.apogee, b.summary.apogee, 1e-9);
    CHECK_NEAR(a.bodies.back().landing.position.x, b.bodies.back().landing.position.x, 1e-6);
    Project p{"t", r, env, SimulationOptions{}, "out", 0.0};
    const Project p2 = Project::fromJson(p.toJson());
    CHECK_NEAR(simulate(p2.rocket, p2.environment, p2.options).summary.apogee, a.summary.apogee, 1e-9);
}

TEST_CASE("triggers: builtins, combinators, latch delay and JSON") {
    FlightSnapshot s;
    SensorReadings sens;
    EventLog log;
    ExternalCommands ext;
    const std::string body = "b";
    const TriggerContext c{s, sens, log, ext, body};
    s.time = 10.0;
    s.altitude = 500.0;
    CHECK(!Trigger::apogee()(c));
    log.add(FlightEvent{8.0, EventType::Apogee, "b"});
    CHECK(Trigger::apogee()(c));
    CHECK(Trigger::apogee(2.0)(c));
    CHECK(!Trigger::apogee(2.5)(c));
    CHECK(Trigger::altitudeBelow(600.0)(c));
    CHECK(!(Trigger::altitudeBelow(600.0) && Trigger::external("main"))(c));
    ext.fire("main");
    CHECK((Trigger::altitudeBelow(600.0) && Trigger::external("main"))(c));
    CHECK((Trigger::never() || Trigger::atTime(5.0))(c));
    CHECK(!(!Trigger::atTime(5.0))(c));
    const Trigger t = Trigger::altitudeBelow(300.0) || Trigger::atTime(100.0).delayed(1.5);
    const Trigger back = Trigger::fromJson(t.toJson());
    CHECK(back.toJson() == t.toJson());
    CHECK_THROWS(Trigger::fromJson(json{{"type", "custom"}, {"name", "not registered"}}));
    const Trigger fb = Trigger::fromJson(
        Trigger::custom("unknown fn", [](const TriggerContext&) { return true; }).withFallback(Trigger::apogee()).toJson());
    CHECK(fb(c));
}
