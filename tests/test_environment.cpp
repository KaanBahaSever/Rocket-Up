#include <nlohmann/json.hpp>
#include <rocketup/RocketUp.hpp>

#include "TestFramework.hpp"

using namespace rocketup;

TEST_CASE("environment: US Standard Atmosphere 1976 reference values") {
    const Planet earth = planets::earth();
    struct Row {
        double h, T, p, rho;
    };
    // Geometric altitude, from the published 1976 tables.
    const Row rows[] = {{0, 288.150, 101325.0, 1.2250},     {1000, 281.651, 89874.6, 1.1117},
                        {5000, 255.676, 54019.9, 0.73612},  {11000, 216.774, 22699.9, 0.36480},
                        {20000, 216.650, 5529.3, 0.088910}, {32000, 228.490, 889.06, 0.013555}};
    for (const auto& r : rows) {
        const AtmosphereState s = earth.atmosphere().at(r.h);
        CHECK_REL(s.temperature, r.T, 2e-4);
        CHECK_REL(s.pressure, r.p, 2e-3);
        CHECK_REL(s.density, r.rho, 2e-3);
    }
    CHECK_NEAR(earth.atmosphere().at(0).speedOfSound, 340.29, 0.05);
    CHECK_NEAR(earth.atmosphere().at(0).dynamicViscosity, 1.789e-5, 2e-8);
}

TEST_CASE("environment: launch-site calibration and offsets") {
    LayeredAtmosphere a = LayeredAtmosphere::usStandard1976();
    a.setConditionsAt(970.0, 273.15 + 30.0, 90000.0);
    const AtmosphereState s = a.at(970.0);
    CHECK_NEAR(s.temperature, 303.15, 1e-6);
    CHECK_NEAR(s.pressure, 90000.0, 1e-6);
    LayeredAtmosphere b = LayeredAtmosphere::usStandard1976();
    b.setTemperatureOffset(10.0);
    CHECK_NEAR(b.at(0).temperature, 298.15, 1e-9);
    CHECK_NEAR(b.at(0).pressure, 101325.0, 1e-6);
}

TEST_CASE("environment: planets from JSON round trip") {
    for (const std::string n : {"earth", "mars", "moon"}) {
        const Planet p = planets::byName(n);
        const Planet q = Planet::fromJson(p.toJson());
        CHECK_NEAR(q.surfaceGravity(), p.surfaceGravity(), 1e-12);
        for (double h : {0.0, 3000.0, 15000.0})
            CHECK_NEAR(q.atmosphere().at(h).pressure, p.atmosphere().at(h).pressure, 1e-6 * (1 + p.atmosphere().at(h).pressure));
    }
    const std::string data = ROCKETUP_DATA_DIR;
    const Planet titan = Planet::loadFile(data + "/planets/titan.json");
    CHECK_NEAR(titan.surfaceGravity(), 1.354, 0.01);
    CHECK_NEAR(titan.atmosphere().at(0).density, 5.2, 0.3);
    const Planet mars = Planet::loadFile(data + "/planets/mars.json");
    CHECK_NEAR(mars.atmosphere().at(0).density, 0.0153, 0.0005);  // NASA Glenn: 0.699/(0.1921*242.1)
    CHECK_THROWS(planets::byName("pluto"));
}

TEST_CASE("environment: gravity, wind and rail geometry") {
    Environment env(planets::earth(), LaunchSite{"", 45.0, 0.0, 0.0});
    CHECK_NEAR(env.gravityAt(0.0), 9.806, 0.003);  // includes centrifugal relief
    CHECK(env.gravityAt(10000.0) < env.gravityAt(0.0));
    const ConstantWind w(10.0, 270.0);  // from the west -> blows to the east
    CHECK_NEAR(w.velocity(0, 0).x, 10.0, 1e-9);
    CHECK_NEAR(w.velocity(0, 0).y, 0.0, 1e-9);
    const PowerLawWind pl(5.0, 0.0, 10.0, 1.0 / 7.0);
    CHECK_NEAR(pl.velocity(10.0, 0).norm(), 5.0, 1e-9);
    CHECK(pl.velocity(1000.0, 0).norm() > 9.0);
    const TurbulentWind tw(std::make_unique<ConstantWind>(8.0, 90.0), 0.2, 3);
    double sum = 0, sum2 = 0;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const double s = tw.velocity(100.0, i * 0.05).x;
        sum += s;
        sum2 += s * s;
    }
    const double mean = sum / n, sd = std::sqrt(sum2 / n - mean * mean);
    CHECK_NEAR(mean, -8.0, 0.4);
    CHECK_NEAR(sd, 1.6, 0.5);
    LaunchRail r;
    r.elevation = 90.0;
    CHECK_NEAR(r.direction().z, 1.0, 1e-12);
    r.elevation = 80.0;
    r.azimuth = 90.0;
    CHECK_NEAR(r.direction().x, std::cos(mathrix::deg2rad(80.0)), 1e-12);
    const Environment e2 = Environment::fromJson(env.toJson());
    CHECK_NEAR(e2.site().latitude, 45.0, 1e-12);
}
