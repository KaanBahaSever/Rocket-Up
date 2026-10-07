#include <nlohmann/json.hpp>
#include <rocketup/RocketUp.hpp>

#include "TestFramework.hpp"

using namespace rocketup;
using mathrix::kPi;

TEST_CASE("rocket: component mass properties") {
    BodyTube t("Tube", 1.0, 0.05, 0.002);
    t.setMaterial(materials::fiberglass());
    const double expected = 1850.0 * kPi * (0.05 * 0.05 - 0.048 * 0.048) * 1.0;
    CHECK_REL(t.mass().mass, expected, 5e-3);
    CHECK_NEAR(t.mass().cg, 0.5, 1e-9);
    NoseCone cone("Cone", NoseShape::Conical, 0.3, 0.05);
    cone.setFilled(true).setMaterial(materials::pla());
    CHECK_REL(cone.mass().mass, 1250.0 * kPi * 0.05 * 0.05 * 0.3 / 3.0, 1e-3);
    CHECK_NEAR(cone.mass().cg, 0.75 * 0.3, 1e-3);  // solid cone CG at 3/4 length from the tip
    CHECK_REL(cone.volume(), kPi * 0.05 * 0.05 * 0.3 / 3.0, 1e-4);
    CHECK_REL(cone.wettedArea(), kPi * 0.05 * std::hypot(0.05, 0.3), 1e-4);
    t.overrideMass(2.0);
    CHECK_NEAR(t.mass().mass, 2.0, 1e-12);
    MassObject m("m", 1.0, 0.1, 0.02);
    m.overrideCg(0.07);
    CHECK_NEAR(m.mass().cg, 0.07, 1e-12);
}

TEST_CASE("rocket: placement and totals") {
    Rocket r("Test");
    r.add<NoseCone>("Nose", NoseShape::Ogive, 0.3, 0.05);
    auto& body = r.add<BodyTube>("Body", 1.0, 0.05, 0.002);
    auto& a = body.add<MassObject>("A", 1.0, 0.1, 0.02);
    a.setPosition(Anchor::Top, 0.2);
    auto& b = body.add<MassObject>("B", 1.0, 0.1, 0.02);
    b.setPosition(Anchor::Bottom, 0.0);
    CHECK_NEAR(r.length(), 1.3, 1e-12);
    double xa = -1, xb = -1;
    for (const auto& p : r.flatten()) {
        if (p.component == &a) xa = p.position;
        if (p.component == &b) xb = p.position;
    }
    CHECK_NEAR(xa, 0.5, 1e-12);
    CHECK_NEAR(xb, 1.2, 1e-12);
    const MassProperties mp = r.massProperties();
    double m = 0, mx = 0;
    for (const auto& p : r.flatten()) {
        const MassProperties c = p.component->mass().shifted(p.position);
        m += c.mass;
        mx += c.mass * c.cg;
    }
    CHECK_NEAR(mp.mass, m, 1e-12);
    CHECK_NEAR(mp.cg, mx / m, 1e-12);
    r.overrideDryMass(5.0, 0.8);
    CHECK_NEAR(r.massProperties().mass, 5.0, 1e-12);
    CHECK_NEAR(r.massProperties().cg, 0.8, 1e-12);
    CHECK_THROWS(body.add<NoseCone>());  // body components belong in the stack
    CHECK_THROWS(a.add<MassObject>());   // mass objects hold nothing
}

TEST_CASE("aero: Barrowman normal force and centre of pressure") {
    // Nose cones: CNa = 2, cone CP at 2/3 L, tangent ogive close to 0.466 L.
    for (auto shape : {NoseShape::Conical, NoseShape::Ogive, NoseShape::Haack, NoseShape::PowerSeries}) {
        Rocket r;
        r.add<NoseCone>("Nose", shape, 0.3, 0.05, 0.5);
        r.add<BodyTube>("Body", 1.0, 0.05, 0.002);
        const AeroModel a(r);
        FlightConditions c;
        c.mach = 0.0;
        c.airspeed = 1.0;
        const auto ac = a.compute(c, 0.5);
        CHECK_NEAR(ac.cnAlpha, 2.0, 1e-9);
        if (shape == NoseShape::Conical) CHECK_NEAR(ac.cp, 0.2, 1e-4);
        if (shape == NoseShape::Ogive) CHECK_NEAR(ac.cp / 0.3, 0.466, 0.01);
    }
    // Classic Barrowman fin equation (incompressible).
    Rocket r;
    r.add<NoseCone>("Nose", NoseShape::Ogive, 0.25, 0.025);
    auto& body = r.add<BodyTube>("Body", 0.6, 0.025, 0.001);
    const double Cr = 0.08, Ct = 0.04, S = 0.06, sweep = 0.04;
    body.addChild(std::make_unique<FinSet>(FinSet::trapezoidal("Fins", 4, Cr, Ct, S, sweep, 0.003)))
        .setPosition(Anchor::Bottom, 0.0);
    const AeroModel a(r);
    FlightConditions c;
    c.mach = 0.0;
    c.airspeed = 1.0;
    const auto br = a.breakdown(c);
    double finCna = 0, finCp = 0;
    for (const auto& x : br)
        if (x.name == "Fins") {
            finCna = x.cnAlpha;
            finCp = x.cp;
        }
    const double d = 0.05, R = 0.025;
    const double lf = std::hypot(S, sweep + Ct / 2 - Cr / 2);
    const double expected = (1 + R / (S + R)) * (4 * 4 * (S / d) * (S / d)) / (1 + std::sqrt(1 + std::pow(2 * lf / (Cr + Ct), 2)));
    CHECK_REL(finCna, expected, 2e-3);
    const double xf = 0.85 - Cr;  // root leading edge
    const double expectedCp =
        xf + sweep / 3 * (Cr + 2 * Ct) / (Cr + Ct) + (Cr + Ct - Cr * Ct / (Cr + Ct)) / 6.0;
    CHECK_NEAR(finCp, expectedCp, 1.5e-3);
}

TEST_CASE("aero: drag build-up correlations and OpenRocket comparison") {
    CHECK_NEAR(AeroModel::baseCd(0.5), 0.12 + 0.13 * 0.25, 1e-12);
    CHECK_NEAR(AeroModel::baseCd(2.0), 0.125, 1e-12);
    CHECK_NEAR(AeroModel::stagnationCd(0.0), 0.85, 1e-12);
    // Turbulent flat plate Cf ~ 0.0037 at Re = 2.3e6, roughness limited for 60 um paint on 2 m.
    CHECK_NEAR(AeroModel::skinFrictionCoefficient(2.3e6, 0.0, 0.0, 2.0), 0.00373, 5e-5);
    CHECK_NEAR(AeroModel::skinFrictionCoefficient(2.3e6, 0.0, 60e-6, 2.0), 0.00398, 5e-5);
    // Orbit Chaser geometry: OpenRocket 15.03 (same file) gives Cd 0.4815 at M 0.315 and
    // 0.5509 at M 0.787 (friction 0.2489, pressure 0.1014, base 0.2006).
    Rocket r = Rocket::load(std::string(ROCKETUP_DATA_DIR) + "/rockets/orbit_chaser.rocketup");
    r.aero().plumeReducesBaseDrag = false;
    const AeroModel a(r);
    const AtmosphereState s = planets::earth().atmosphere().at(2500.0);
    FlightConditions c;
    c.density = s.density;
    c.kinematicViscosity = s.kinematicViscosity();
    c.mach = 0.315;
    c.airspeed = c.mach * s.speedOfSound;
    CHECK_REL(a.compute(c, 0.97).cd, 0.4815, 0.03);
    c.mach = 0.787;
    c.airspeed = c.mach * s.speedOfSound;
    const auto hi = a.compute(c, 0.97);
    CHECK_REL(hi.cd, 0.5509, 0.03);
    CHECK_REL(hi.cdFriction, 0.2489, 0.05);
    CHECK_REL(hi.cdBase, 0.2006, 0.02);
    // Table drag source replaces the total but keeps the stability model.
    r.aero().setTable({0.0, 1.0}, {0.30, 0.50});
    const AeroModel t(r);
    CHECK_NEAR(t.compute(c, 0.97).cd, 0.30 + 0.20 * 0.787, 1e-9);
    CHECK_NEAR(t.compute(c, 0.97).cp, hi.cp, 1e-9);
}

TEST_CASE("rocket: design file round trip") {
    const std::string path = std::string(ROCKETUP_DATA_DIR) + "/rockets/orbit_chaser.rocketup";
    const Rocket r = Rocket::load(path);
    CHECK(r.stack().size() == 4);
    CHECK(r.validate().empty());
    const Rocket q = Rocket::fromJson(r.toJson(), io::directoryOf(path));
    CHECK(q.toJson() == r.toJson());
    CHECK_NEAR(q.massProperties().mass, r.massProperties().mass, 1e-12);
    CHECK_NEAR(q.massProperties().cg, r.massProperties().cg, 1e-12);
    const Rocket copy = r;  // deep copy
    CHECK(copy.toJson() == r.toJson());
    CHECK(copy.find("Main") != r.find("Main"));
}
