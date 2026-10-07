# Architecture

Rocket-Up is split into a header-only math library ([MathRix](https://github.com/KaanBahaSever/MathRix), git submodule), the simulation library (`rocketup`) and thin front-ends: the
command line tool, the examples and, later, a GUI. Design objects are plain data. Simulations copy them and keep all runtime state
to themselves, so one design can be simulated many times in parallel. A GUI can also edit a design while a simulation runs.

```
MathRix  (separate repository, header-only, no dependencies)
  Vec3, Mat3, Quat, Matrix, Polynomial, VecN, Interpolator1D (step/linear/spline/akima/pchip), Table2D, ODE steppers

rocketup
  core/          Types (Vec3, json, constants), Io (files, CSV parsing, paths)
  environment/   AtmosphereModel <- LayeredAtmosphere | TableAtmosphere | ExponentialAtmosphere | VacuumAtmosphere
                 Planet (GM, radius, rotation, gravity model, atmosphere)
                 WindModel <- NoWind | ConstantWind | PowerLawWind | TableWind | TurbulentWind | TransformedWind
                 LaunchSite, LaunchRail, Environment
  propulsion/    Motor (thrust curve, mass model, pressure correction), MotorLoader (.eng/.rse/.csv/.rumotor)
  components/    Component (tree, placement, mass, JSON)
                   SymmetricComponent <- NoseCone | BodyTube | Transition
                   FinSet (polygon outline), LaunchLug, RailButton
                   InnerTube <- MotorMount, Ring, MassObject <- Payload
                   RecoveryDevice <- Parachute | Streamer
                 Material, SurfaceFinish
  rocket/        Rocket (stack, separations, AeroSettings, dry-mass override, JSON)
  aero/          AeroModel (Barrowman + drag build-up), FlightConditions, AeroCoefficients
  sim/           Trigger, Events, FlightState, Sensors, Simulation, Results
  io/            CSV / JSON / KML export, HTML report, SVG drawing, Project files
  hil/           SerialPort, protocol, HilBridge (SimulationObserver), Links
  analysis/      Monte Carlo dispersion
```

## Components

Every part derives from `Component`. A component has a name, a placement inside its parent (`Anchor::Top|Middle|Bottom` plus an
offset, as in OpenRocket), optional mass/CG overrides and children. It also implements:

- `computeMass()`: mass, CG and inertia in its local frame, measured from the component's front;
- `writeProperties()` / `readProperties()`: its JSON fields;
- `clone()`: a deep copy (the `ROCKETUP_COMPONENT_CLONE` macro).

`Rocket` owns the *stack* of axisymmetric body components (nose to tail) and lays them end to end. `Rocket::flatten()` returns every
component with its absolute position from the nose tip. Mass, aerodynamics and drawings all work from this flat list.

New component types are added by deriving from `Component` (or one of its subclasses) and calling `Component::registerType("my-type", factory)`.
They then load from design files like the built-in ones.

## Simulation loop

`Simulation::run()` creates a private `Engine` that flies a queue of *bodies*. The first body is the whole rocket. A body holds:

- its list of placed components and the static structure mass computed once;
- an `AeroModel` built for exactly those components;
- the runtime state of its motors (ignition time), recovery devices (deployment time, peak load), payloads and separations;
- a 13-element state vector: ENU position and velocity, attitude quaternion (body to ENU) and body angular rate;
- a flight mode: `Constrained` (launch rail), `Free6` (6-DOF), `Free3` (point mass aligned with the relative wind) or `Descent`
  (point mass with parachute and tumbling drag);
- its own event log, sensor simulator and recorded history.

Each step does the following:

1. Integrate with the selected scheme. RK45 adapts the step size within `[minTimeStep, maxTimeStep]`.
2. Apply the rail constraint and detect rail exit, then detect ground contact and interpolate to `z = 0`.
3. Detect apogee (interpolated from the sign change of the vertical velocity) and burnout.
4. Update the flight snapshot and the simulated sensors (barometer, accelerometer, gyroscope, GPS) at their own rates.
5. Poll observers for external commands (HIL), then evaluate every trigger. Ignitions, deployments, payload ejections and separations are
   executed. An ejection or separation removes components from the body and pushes new bodies onto the queue. The new bodies inherit
   the state and the event history.
6. Record the sample and notify observers.

Bodies never interact after they separate, so each one is flown to the ground on its own. This is also what makes
hardware-in-the-loop simple: the body that carries the flight computer is integrated first, and in real time if requested.

### Forces and moments (6-DOF)

- Thrust acts along the body x axis. It is corrected for ambient pressure when the nozzle exit area is known.
- Aerodynamic axial force is `−q A C_A(α)` and the normal force is `q A C_N` at the CP, opposing the lateral relative wind.
- The pitch and yaw damping moment is `−½ ρ ω⊥|ω⊥| A d C_damp`, using OpenRocket's damping multiplier.
- The roll moment is `q A d (C_lf − C_ld)`, from fin cant forcing and strip-theory damping.
- Gravity is `−(GM/r² − Ω²r cos²φ) ẑ`. The optional Coriolis term is `−2 Ω × v`.
- Euler's equations use the axial and transverse inertia of the current configuration, which changes as propellant burns.

### Triggers

A `Trigger` wraps `std::function<bool(const TriggerContext&)>`. The context gives read-only access to the true state, the simulated
sensor readings, the body's event log and the external command channels. Builtin factories (`apogee`, `altitudeBelow`, `burnout`,
`afterEvent`, `baroApogee`, `external`, ...) also know how to serialize themselves.

- A custom lambda is stored by name in design files. It is re-attached with `Trigger::registerNamed`, or replaced by the builtin
  `withFallback(...)` trigger when the program loading the file does not know the lambda.
- `delayed(seconds)` latches the condition and fires later. The latch lives in the simulation, so triggers stay stateless and thread safe.

## Threading

Designs (`Rocket`, `Environment`) are copied into each `Simulation`. Runs share nothing mutable apart from the trigger registry,
which is protected by a mutex. `runMonteCarlo` uses this to run one simulation per hardware thread.

## Notes for a future GUI

- **Editing.** Every component, motor, planet and environment round-trips through JSON, so property editors can be generated
  from the JSON fields. `Rocket::validate()` reports design problems.
- **Drawing.** `io::drawRocketSvg()` returns a side view that follows the theme color (`currentColor`). `Component::outerRadius()`,
  `SymmetricComponent::radiusAt()` and `FinSet::outline()` give exact geometry for a custom renderer.
- **Live plots.** A `SimulationObserver` receives every step (`onStep`) and every event (`onEvent`).
- **Responsiveness.** `Simulation::setProgressCallback()` reports progress and `Simulation::cancel()` stops a run from any thread.
- **Tables.** `recordColumns()` describes every recorded quantity with a key, label, unit and accessor, ready for tables and axis pickers.
- **Stability readout.** `AeroModel::breakdown()` gives the per-component CNα, CP and drag needed for a live stability display.
