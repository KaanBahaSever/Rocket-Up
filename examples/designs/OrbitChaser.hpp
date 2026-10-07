#pragma once

// Orbit Chaser - the reference design of Rocket-Up.
//
// Geometry and masses follow the Istanbul University Rocket Team's TEKNOFEST 2021
// mid-altitude rocket (130 mm fiberglass airframe, Cesaroni Pro75 M-class motor), so the
// results can be compared directly with the team's OpenRocket model.

#include <rocketup/RocketUp.hpp>

namespace designs {

inline rocketup::Rocket buildOrbitChaser(const rocketup::Motor& motor) {
    using namespace rocketup;
    Rocket rocket("Orbit Chaser");
    rocket.setDesigner("Kaan Baha Sever");
    rocket.setDescription("130 mm dual-deploy rocket with an ejectable payload, M-class motor");

    const double R = 0.065;  // body radius (130 mm)

    // ---------------------------------------------------------------- nose cone
    auto& nose = rocket.add<NoseCone>("Nose cone", NoseShape::PowerSeries, 0.12, R, 0.5);
    nose.setThickness(0.006).setMaterial(materials::fiberglass());
    nose.setShoulder(0.195, 0.0625, 0.0025);
    nose.add<MassObject>("Nose eyebolt", 0.1, 0.025, 0.0125, "recovery").setPosition(Anchor::Top, 0.005);

    auto& payload = nose.add<Payload>("Payload", 4.025, 0.112, 0.04);
    payload.setPosition(Anchor::Top, 0.20);
    payload.setEjectTrigger(Trigger::apogee(1.0));  // pushed out one second after apogee
    payload.setEjectionSpeed(3.0);
    auto& payloadChute = payload.add<Parachute>("Payload parachute", 1.2, 0.8);
    payloadChute.setDeployTrigger(Trigger::afterEvent(EventType::Ejection, 1.0, "Payload"));
    payloadChute.setLines(6, 1.0);

    // ---------------------------------------------------------------- body 1: drogue + avionics
    auto& body1 = rocket.add<BodyTube>("Body 1", 0.505, R, 0.0025);
    body1.overrideMass(0.926);
    auto& drogue = body1.add<Parachute>("Drogue", 0.8, 0.8);
    drogue.setPosition(Anchor::Top, 0.0);
    // A flight-computer style trigger written as a plain C++ lambda: barometric apogee
    // detection on the simulated (noisy) sensor data, with a mach lock-out.
    drogue.setDeployTrigger(Trigger::custom("baro apogee (5 m drop, mach < 0.3)", [](const TriggerContext& c) {
        return c.sensors.valid && c.state.mach < 0.3 && c.sensors.maxBaroAltitude > 100.0 &&
               c.sensors.maxBaroAltitude - c.sensors.baroAltitude > 5.0;
    }).withFallback(Trigger::baroApogee(5.0)));
    body1.add<InnerTube>("Rail guide 1", 0.21, 0.0625, 0.0025).setPosition(Anchor::Bottom, -0.1).overrideMass(0.3927);
    body1.add<MassObject>("Spring ejector 1", 0.755, 0.08, 0.04, "recovery").setPosition(Anchor::Top, 0.32);
    body1.add<MassObject>("Eyebolt body 1", 0.1, 0.025, 0.0125, "recovery").setPosition(Anchor::Top, 0.38);
    body1.add<MassObject>("Eyebolt separation 1", 0.1, 0.025, 0.0125, "recovery").setPosition(Anchor::Top, 0.31);

    auto& avBay = body1.add<InnerTube>("Avionics bay", 0.2, 0.0625, 0.0025);
    avBay.setPosition(Anchor::Bottom, 0.1).overrideMass(0.188);
    avBay.add<Ring>("Avionics top bulkhead", 0.02).setMaterial(materials::aluminum()).setPosition(Anchor::Bottom, -0.18);
    avBay.add<Ring>("Avionics bottom bulkhead", 0.02).setMaterial(materials::aluminum()).setPosition(Anchor::Bottom, 0.0);
    avBay.add<MassObject>("Flight computer", 0.1, 0.105, 0.04, "avionics").setPosition(Anchor::Top, 0.045);
    avBay.add<MassObject>("Backup flight computer", 0.1, 0.06, 0.025, "avionics").setPosition(Anchor::Top, 0.07);
    avBay.add<MassObject>("Batteries", 0.5, 0.07, 0.035, "battery").setPosition(Anchor::Top, 0.065);

    // ---------------------------------------------------------------- body 2: main chute + ballast
    auto& body2 = rocket.add<BodyTube>("Body 2", 0.36, R, 0.0025);
    body2.overrideMass(0.648);
    body2.add<InnerTube>("Rail guide 2", 0.16, 0.0625, 0.0025).setPosition(Anchor::Bottom, -0.1).overrideMass(0.2571);
    body2.add<MassObject>("Spring ejector 2", 0.755, 0.08, 0.04, "recovery").setPosition(Anchor::Top, 0.1);
    auto& mainChute = body2.add<Parachute>("Main", 2.0, 0.8);
    mainChute.setPosition(Anchor::Top, 0.27);
    mainChute.setDeployTrigger(Trigger::altitudeBelow(600.0));
    mainChute.setLines(6, 2.5);
    body2.add<MassObject>("Eyebolt motor section", 0.1, 0.025, 0.0125, "recovery").setPosition(Anchor::Top, 0.38);
    auto& coupler = body2.add<InnerTube>("Lower coupler", 0.195, 0.0625, 0.0025);
    coupler.setPosition(Anchor::Bottom, 0.0975);
    coupler.add<MassObject>("Ballast", 3.315, 0.041, 0.06, "ballast").setPosition(Anchor::Bottom, 0.0);

    // ---------------------------------------------------------------- body 3: motor section
    auto& body3 = rocket.add<BodyTube>("Body 3", 1.04, R, 0.0025);
    auto& mount = body3.add<MotorMount>("Motor mount", 0.905, 0.04, 0.0025);
    mount.setPosition(Anchor::Bottom, 0.0);
    mount.setMotor(motor);
    auto& fins = body3.addChild(std::make_unique<FinSet>(FinSet::freeform(
        "Fins", 4, {{0.0, 0.0}, {0.0, 0.1}, {0.06, 0.08}, {0.08, 0.05}, {0.1, 0.0}}, 0.0025)));
    static_cast<FinSet&>(fins).setMaterial(materials::aluminum()).setCrossSection(FinCrossSection::Square);
    fins.setPosition(Anchor::Bottom, 0.0).overrideMass(0.3918);
    body3.add<Ring>("Motor retainer (fwd)", 0.0495, -1.0, 0.04).setRole("centering-ring").setMaterial(materials::aluminum())
        .setPosition(Anchor::Bottom, -0.893).overrideMass(1.238);
    body3.add<Ring>("Motor retainer (mid)", 0.03, -1.0, 0.04).setRole("centering-ring").setMaterial(materials::aluminum())
        .setPosition(Anchor::Bottom, -0.17);
    body3.add<Ring>("Motor retainer (aft)", 0.0565, -1.0, 0.0375).setRole("engine-block").setMaterial(materials::aluminum())
        .setPosition(Anchor::Bottom, 0.0265).overrideMass(0.689);
    body3.add<LaunchLug>("Upper rail lug", 0.02, 0.005, 0.003).setPosition(Anchor::Middle, -0.673);
    body3.add<LaunchLug>("Lower rail lug", 0.02, 0.005, 0.003).setPosition(Anchor::Middle, -0.186);

    // Weighed on the scale: 21.23 kg without motor, CG 0.886 m from the nose tip.
    rocket.overrideDryMass(21.229, 0.886);
    return rocket;
}

}  // namespace designs
