// Choosing the thrust-curve interpolation of a motor.
//
// Thrust curves are sparse samples (the M2150 file has 15 points). How the simulator fills the
// gaps changes the impulse, the peak thrust and therefore the flight a little. Every motor can
// use any scheme:
//
//   C++:      motor.setInterpolation(Interp::Pchip);
//   loading:  MotorLoadOptions o; o.interpolation = Interp::Akima; MotorLoader::load(path, o);
//   .rumotor: "thrustCurve": { "interpolation": "pchip", ... }
//   design:   "motor": { "file": "my.eng", "interpolation": "spline" }
//   CLI:      rocketup motor my.eng --interp pchip --convert my.rumotor
//
// This program compares all of them and writes the curves to output/motor_interpolation.csv.

#include <iomanip>
#include <iostream>
#include <rocketup/RocketUp.hpp>
#include <sstream>

#include "designs/OrbitChaser.hpp"

using namespace rocketup;

int main() {
    const std::string data = ROCKETUP_DATA_DIR;
    try {
        const Motor base = MotorLoader::load(data + "/motors/Cesaroni_7455M2150-P.rumotor");
        const Interp methods[] = {Interp::Step, Interp::Linear, Interp::CubicSpline, Interp::Akima, Interp::Pchip};
        const Environment env(planets::earth(), LaunchSite{"Aksaray", 38.4, 34.0, 970.0}, LaunchRail{6.0, 85.0, 0.0});

        std::cout << base.displayName() << " (" << base.time.size() << " data points)\n\n";
        std::cout << "interpolation   impulse (Ns)   peak (N)   apogee (m)\n";
        std::vector<Motor> motors;
        for (Interp m : methods) {
            Motor motor = base;
            motor.setInterpolation(m);
            Rocket rocket = designs::buildOrbitChaser(motor);
            rocket.overrideDryMass(18.715, 0.8959);
            SimulationOptions o;
            o.stopAtApogee = true;
            const SimulationResult r = simulate(rocket, env, o);
            double peak = 0.0;
            for (double t = 0.0; t <= motor.burnTime(); t += 1e-3) peak = std::max(peak, motor.thrustAt(t));
            std::cout << std::left << std::setw(14) << mathrix::toString(m) << std::right << std::fixed
                      << std::setprecision(1) << std::setw(14) << motor.totalImpulse() << std::setw(11) << peak
                      << std::setw(13) << r.summary.apogee << "\n";
            motors.push_back(motor);
        }

        std::ostringstream csv;
        csv << "time_s";
        for (Interp m : methods) csv << "," << mathrix::toString(m) << "_N";
        csv << "\n" << std::setprecision(6);
        for (double t = 0.0; t <= base.burnTime() + 0.05; t += 0.002) {
            csv << t;
            for (const Motor& m : motors) csv << "," << m.thrustAt(t);
            csv << "\n";
        }
        io::writeTextFile("output/motor_interpolation.csv", csv.str());
        std::cout << "\nCurves written to output/motor_interpolation.csv\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
