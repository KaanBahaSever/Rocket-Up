<p align="center">
  <img src="docs/logo.svg" width="150" alt="Rocket-Up logo">
</p>

<h1 align="center">Rocket-Up</h1>

<p align="center">
  <b>A modern, modular rocket flight simulator in C++17.</b><br>
  Build rockets from components, load any motor, launch from any planet, and get every number and plot you need.
</p>

---

Rocket-Up is an open-source alternative to OpenRocket written as a reusable C++ library with a command line tool.
It started as a Python 3-DOF competition simulator (TEKNOFEST 2021) and was rewritten to be fast, modular,
data driven and ready for a cross-platform GUI.

- **Component tree.** A rocket is a stack of nose cones, body tubes and transitions. Each one holds fins, launch lugs, rail buttons,
  inner tubes, bulkheads, mass objects, motor mounts, parachutes, streamers and payloads.
- **Motors are data.** It reads RASP `.eng`, RockSim `.rse`, ThrustCurve.org `.csv` and its own `.rumotor` JSON format. Missing
  Isp, propellant mass or case mass are derived or assumed, and every assumption is listed as a warning. Thrust interpolation
  can be step, linear, natural spline, Akima or PCHIP. Thrust is corrected for ambient pressure when the nozzle exit area is known.
- **Any planet.** Earth (US Standard Atmosphere 1976), Mars (NASA Glenn) and the Moon are built in. Venus, Titan or your own world
  can be described in JSON with layered, tabulated or exponential atmospheres, any gas, and gravity that is inverse-square or constant.
- **Drag without CFD.** Barrowman normal force and CP plus a Niskanen/OpenRocket-style drag build-up. Skin friction depends on
  Reynolds number and roughness, and nose, shoulder, boattail, fin, lug and base pressure drag are computed separately. The model
  covers Mach and AoA effects and the exhaust plume. You can also supply your own `Cd(Mach)` or `Cd(Mach, altitude)` table, or a constant Cd.
- **Physics.** 6-DOF rigid-body or 3-DOF point-mass dynamics, with a launch-rail constraint, wind (constant, power law, table, or
  turbulent gusts), pitch, yaw and roll damping, fin cant, and optional Coriolis.
- **Integrators.** Euler, midpoint, Heun, RK4 and adaptive Dormand-Prince RK45 (selectable).
- **Triggers are functions.** Deployment, separation, ejection and ignition are driven by builtin triggers (`apogee`, `altitudeBelow`,
  `burnout + 2 s`, ...), by any C++ lambda that reads the flight state or simulated noisy sensors, or by commands from an external
  flight computer.
- **Multi-body recovery.** Separated sections and ejected payloads become independent bodies. Each one is flown to the ground with
  its own parachutes, descent rate, opening shock load, drift and landing energy. Multi-stage rockets use the same mechanism.
- **Hardware-in-the-loop.** Rocket-Up streams simulated barometer, IMU and GPS data over a serial port to an Arduino or any other
  board and fires parachutes from its replies. An Arduino sketch and an in-process emulator are included.
- **Monte Carlo.** Runs perturbed simulations in parallel and draws landing dispersion ellipses for every body.
- **Exports.** CSV with about 50 columns per body, an event list, JSON summary, Google Earth KML, an interactive HTML report with 15 charts
  and a rocket drawing, plus SVG drawings of the design.
- **Save and open.** Rockets (`.rocketup`), environments, planets, motors and whole projects are plain JSON files.

## Validation

The reference rocket *Orbit Chaser* uses the geometry and masses of a TEKNOFEST 2021 mid-altitude rocket. It is compared with the
team's OpenRocket 15.03 models, flown under the same conditions: 6 m/s wind from the south, rail tilted 5° to the north,
launch site at 970 m, and base drag not reduced during the burn.

| | OpenRocket M2020 | Rocket-Up M2020 | OpenRocket M2150 | Rocket-Up M2150 |
|---|---:|---:|---:|---:|
| Apogee | 3111.6 m | **3109.1 m** | 2985.5 m | **2982.0 m** |
| Time to apogee | 25.39 s | 25.37 s | 24.61 s | 24.62 s |
| Max velocity | 263.46 m/s | 263.47 m/s | 272.08 m/s | 272.10 m/s |
| Max Mach | 0.788 | 0.788 | 0.813 | 0.813 |
| Rail exit velocity | 31.95 m/s | 31.97 m/s | 32.59 m/s | 32.60 m/s |
| Max acceleration | 85.7 m/s² | 85.9 m/s² | 98.8 m/s² | 99.4 m/s² |

Along the whole trajectory, altitude, velocity and the drag coefficient stay within 0.5 % of OpenRocket. At Mach 0.79 the drag
coefficient is Cd 0.554 against 0.551, with friction 0.254 / 0.249, pressure 0.100 / 0.101 and base 0.201 / 0.201.
The test suite checks these numbers on every build. More cases are in [docs/VALIDATION.md](docs/VALIDATION.md).

## Quick start

Requirements: a C++17 compiler (GCC 9+, Clang 10+, MSVC 2019+) and CMake 3.16 or newer.
The math library [MathRix](https://github.com/KaanBahaSever/MathRix) is a git submodule (CMake downloads it automatically when
the submodule is missing) and [nlohmann/json](https://github.com/nlohmann/json) is vendored.

```bash
git clone --recursive https://github.com/KaanBahaSever/Rocket-Up.git
cd Rocket-Up
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release            # unit tests + OpenRocket validation
./build/bin/orbit_chaser                     # full example -> output/orbit_chaser/report.html
```

### Command line

```bash
rocketup run data/projects/orbit_chaser.project.json -o output/oc   # simulate + export everything
rocketup run my_rocket.rocketup --integrator rk45 --dynamics 3dof    # override options
rocketup info data/rockets/orbit_chaser.rocketup --svg rocket.svg    # mass, CG, CP, Cd breakdown, drawing
rocketup motor my_motor.csv --isp 195 --total 2.1 --convert my_motor.rumotor
rocketup planet mars --table 20000                                   # atmosphere table of any planet
rocketup montecarlo data/projects/orbit_chaser.project.json --runs 500
rocketup hil data/projects/orbit_chaser_hil.project.json --port COM3 # Arduino in the loop
```

`tools/thrustcurve_fetch.py M2020 --manufacturer Cesaroni --convert build/bin/rocketup` downloads the best available data file
from ThrustCurve.org. It prefers certification data, and RockSim files because they include the mass curve. The script then converts the file to `.rumotor`.

## Using the library

```cpp
#include <rocketup/RocketUp.hpp>
using namespace rocketup;

int main() {
    Motor motor = MotorLoader::load("data/motors/Cesaroni_8429M2020-P.rumotor");
    motor.setInterpolation(Interp::Pchip);   // Step, Linear (default), CubicSpline, Akima, Pchip

    Rocket rocket("Orbit Chaser");
    rocket.add<NoseCone>("Nose", NoseShape::Haack, 0.45, 0.065, 0.0);
    auto& body = rocket.add<BodyTube>("Body", 1.9, 0.065, 0.0025);
    body.add<MotorMount>("Mount", 0.9, 0.04, 0.0025).setPosition(Anchor::Bottom, 0.0);
    static_cast<MotorMount&>(*body.children().back()).setMotor(motor);
    body.addChild(std::make_unique<FinSet>(FinSet::trapezoidal("Fins", 4, 0.2, 0.08, 0.12, 0.10, 0.004)))
        .setPosition(Anchor::Bottom, 0.0);

    // Triggers: builtins, combinations, or any C++ function of the flight state / sensors.
    auto& drogue = body.add<Parachute>("Drogue", 0.8);
    drogue.setDeployTrigger(Trigger::custom("baro apogee", [](const TriggerContext& c) {
        return c.sensors.maxBaroAltitude - c.sensors.baroAltitude > 5.0;   // noisy simulated barometer
    }));
    body.add<Parachute>("Main", 2.0).setDeployTrigger(Trigger::altitudeBelow(600.0));
    auto& payload = body.add<Payload>("CanSat", 0.5, 0.11, 0.033);
    payload.setEjectTrigger(Trigger::apogee(1.0));                  // becomes its own falling body
    payload.add<Parachute>("CanSat chute", 0.6).setDeployTrigger(Trigger::afterEvent(EventType::Ejection, 0.5));

    Environment env(planets::earth(), LaunchSite{"Aksaray", 38.4, 34.0, 970.0}, LaunchRail{6.0, 85.0, 0.0},
                    std::make_unique<PowerLawWind>(5.0, 270.0));

    SimulationOptions options;
    options.integrator = Integrator::RK4;              // Euler, Midpoint, Heun, RK4, DormandPrince
    options.dynamics = DynamicsModel::SixDof;          // or ThreeDof

    SimulationResult result = Simulation(rocket, env, options).run();
    io::printSummary(std::cout, result);
    io::exportAll(result, rocket, env, "output/my_flight");   // CSV, JSON, KML, HTML report
    rocket.save("my_flight.rocketup");
}
```

### Thrust-curve interpolation

Each motor chooses how its thrust is interpolated between data points: `Step`, `Linear` (the default, because it reproduces the
certified total impulse), natural `CubicSpline`, `Akima`, or monotone `Pchip` (smooth and never overshoots). The choice can be made
at any of these levels:

| Where | How |
|---|---|
| C++ | `motor.setInterpolation(Interp::Akima);` or `MotorLoadOptions::interpolation` |
| `.rumotor` file | `"thrustCurve": { "interpolation": "pchip", ... }` |
| rocket design | `"motor": { "file": "my_motor.eng", "interpolation": "spline" }` |
| command line | `rocketup motor my_motor.eng --interp pchip --convert my_motor.rumotor` |

With the 15-point M2150 curve, the scheme alone moves the total impulse between 7443 and 7813 Ns and the apogee by up to 190 m
(`examples/motor_interpolation`). Smooth schemes stay within 1.4 % of the certified impulse.

More complete programs are in [`examples/`](examples):

| Example | Shows |
|---|---|
| `orbit_chaser` | Full reference design, measured site weather, turbulent wind, custom trigger, payload ejection, all exports |
| `teknofest_3dof` | The TEKNOFEST 2021 3-DOF data set (constant Isp, Cd table, Euler at 10 ms) compared with RK4 and 6-DOF |
| `other_worlds` | The same rocket on Earth, Mars, the Moon, Titan and Venus |
| `two_stage` | Staging through a separation event and a triggered sustainer ignition, where both stages are recovered |
| `hil_flight_computer` | Hardware-in-the-loop with an emulated or real flight computer; see [`examples/arduino`](examples/arduino/RocketUpHIL/RocketUpHIL.ino) |
| `monte_carlo` | Landing dispersion ellipses for the rocket and the payload |
| `load_design` | Opening a saved project, registering custom triggers, editing and re-running |
| `motor_interpolation` | Step, linear, spline, Akima and PCHIP thrust curves compared: impulse, peak and apogee |

## Documentation

- [Architecture](docs/ARCHITECTURE.md): modules, class hierarchy, simulation loop, threading and GUI notes
- [Aerodynamics](docs/AERODYNAMICS.md): how Cd, CNα and CP are computed, references, and how to use your own data
- [File formats](docs/FILE_FORMATS.md): `.rumotor`, `.rocketup`, planets, environments, projects, CSV columns
- [Hardware in the loop](docs/HIL.md): serial protocol and Arduino setup
- [Validation](docs/VALIDATION.md): comparisons with OpenRocket, analytic cases and the old Python simulator

## Repository layout

```
include/rocketup/   public headers (components, propulsion, environment, aero, sim, io, hil, analysis)
src/                library implementation
libs/MathRix/       math library (git submodule): vectors, quaternions, matrices, interpolation, ODE solvers
apps/               the `rocketup` command line tool
examples/           example programs, the Orbit Chaser design, Arduino sketch
data/               motors (+ original source files), planets, Cd tables, rocket designs, projects
tests/              unit and validation tests (ctest)
tools/              ThrustCurve.org downloader
docs/               documentation and logo
```

## Roadmap

- Cross-platform GUI (Qt 6 or Dear ImGui): the library already exposes everything it needs, including component JSON, SVG drawings,
  observers for live plots and cancellable runs on background threads
- Flight-computer filters live in [Rocket-Up-Filters](https://github.com/KaanBahaSever/Rocket-Up-Filters): a Kalman altitude
  estimator, launch and apogee detectors that run on Arduino and can be tested against Rocket-Up through the HIL bridge
- OpenRocket `.ork` import, motor clusters, pods and side boosters
- Fin flutter and structural load estimates, plus a thrust-misalignment dispersion term

## References

- J. S. Barrowman, *The Practical Calculation of the Aerodynamic Characteristics of Slender Finned Vehicles*, 1967
- S. Niskanen, *Development of an Open Source Model Rocket Simulation Software*, Helsinki University of Technology, 2009
- S. F. Hoerner, *Fluid-Dynamic Drag*, 1965
- *U.S. Standard Atmosphere*, NOAA/NASA/USAF, 1976; NASA Glenn Research Center Mars atmosphere model
- Motor data: [ThrustCurve.org](https://www.thrustcurve.org) (attribution in each `.rumotor` file)

## License

MIT. See [LICENSE](LICENSE). Bundled: nlohmann/json (MIT).
