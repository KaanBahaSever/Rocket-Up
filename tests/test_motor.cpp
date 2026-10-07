#include <nlohmann/json.hpp>
#include <rocketup/RocketUp.hpp>

#include "TestFramework.hpp"

using namespace rocketup;

namespace {
const std::string data = ROCKETUP_DATA_DIR;
}

TEST_CASE("motor: every input format gives the same certified curve") {
    const Motor eng = MotorLoader::load(data + "/motors/source/Cesaroni_8429M2020-P.eng");
    const Motor rse = MotorLoader::load(data + "/motors/source/Cesaroni_8429M2020-P.rse");
    const Motor ru = MotorLoader::load(data + "/motors/Cesaroni_8429M2020-P.rumotor");
    for (const Motor* m : {&eng, &rse, &ru}) {
        CHECK_NEAR(m->totalImpulse(), 8428.7, 1.0);  // ThrustCurve.org certified value
        CHECK_NEAR(m->burnTime(), 4.301, 1e-9);
        CHECK_NEAR(m->propellantMass, 4.349, 1e-9);
        CHECK_NEAR(m->totalMass, 7.0318, 1e-9);
        CHECK_NEAR(m->diameter, 0.075, 1e-12);
        CHECK(m->impulseClass() == "M");
        CHECK_NEAR(m->effectiveIsp(), 197.6, 0.2);
    }
    CHECK(eng.manufacturer == "Cesaroni Technology");
    // A bare ThrustCurve CSV has no masses: completed with documented assumptions.
    const Motor csv = MotorLoader::load(data + "/motors/source/Cesaroni_8429M2020-P_thrustcurve.csv");
    CHECK_NEAR(csv.totalImpulse(), 8428.7, 1.0);
    CHECK(csv.warnings().size() >= 2);
    MotorLoadOptions o;
    o.isp = 197.6;
    o.totalMass = 7.03;
    const Motor csv2 = MotorLoader::load(data + "/motors/source/Cesaroni_8429M2020-P_thrustcurve.csv", o);
    CHECK_NEAR(csv2.propellantMass, 4.349, 0.01);  // derived from Isp
}

TEST_CASE("motor: mass flow conserves propellant and pressure thrust") {
    Motor m = MotorLoader::load(data + "/motors/Cesaroni_7455M2150-P.rumotor");
    CHECK_NEAR(m.propellantMassAt(0.0), m.propellantMass, 1e-12);
    CHECK_NEAR(m.propellantMassAt(m.burnTime()), 0.0, 1e-9);
    // Integral of mdot equals the propellant mass.
    double burned = 0.0;
    for (double t = 0.0; t < m.burnTime(); t += 1e-4) burned += m.massFlowAt(t + 5e-5) * 1e-4;
    CHECK_NEAR(burned, m.propellantMass, 2e-3);
    // Ambient-pressure correction only with a known nozzle.
    const double f0 = m.thrustAt(1.0, 101325.0);
    CHECK_NEAR(m.thrustAt(1.0, 0.0), f0, 1e-9);
    m.nozzleExitArea = 0.002;
    m.finalize();
    CHECK_NEAR(m.thrustAt(1.0, 0.0) - m.thrustAt(1.0, 101325.0), 101325.0 * 0.002, 1e-6);
    // Constant-Isp consumption (competition rule).
    Motor c = m;
    c.massModel = MassFlowModel::ConstantIsp;
    c.isp = 180.0;
    c.finalize();
    CHECK(c.propellantMassAt(c.burnTime()) <= 1e-9);
    for (mathrix::Interp i : {mathrix::Interp::CubicSpline, mathrix::Interp::Akima, mathrix::Interp::Pchip}) {
        Motor s = m;
        s.interpolation = i;
        s.finalize();
        CHECK_REL(s.totalImpulse(), m.totalImpulse(), 0.02);
        CHECK(s.thrustAt(2.0) > 0.0);
    }
}

TEST_CASE("motor: rumotor JSON round trip and bad input") {
    const Motor m = MotorLoader::load(data + "/motors/source/Cesaroni_7455M2150-P.rse");
    const Motor r = Motor::fromJson(m.toJson());
    CHECK_NEAR(r.totalImpulse(), m.totalImpulse(), 1e-9);
    CHECK_NEAR(r.massAt(1.0), m.massAt(1.0), 1e-9);
    CHECK_THROWS(MotorLoader::parseEng("this is not a motor"));
    Motor empty;
    CHECK_THROWS(empty.finalize());
}
