# File formats

Every Rocket-Up file is JSON (comments allowed) and uses SI units: m, kg, s, N, Pa, K. Angles are in degrees.
Radii may be given as `"radius"` or `"diameter"` (and `foreRadius`/`foreDiameter`, ...). Relative paths are resolved
against the directory of the file that contains them.

| Extension | Content | Format tag |
|---|---|---|
| `.rumotor` | one motor | `rocketup-motor` |
| `.rocketup` | one rocket design | `rocketup-rocket` |
| `*.json` in `data/planets` | a planet or moon | `rocketup-planet` |
| `*.project.json` | rocket + environment + simulation options + output | `rocketup-project` |

## Motors (`.rumotor`)

Thrust-curve files found online are inconsistent. RASP `.eng` files have masses but no Isp. ThrustCurve CSV files have neither
masses nor dimensions. RockSim `.rse` files have a mass curve. The RocketUp format stores everything explicitly and keeps the source
for attribution:

```json
{
  "format": "rocketup-motor", "version": 1,
  "designation": "8429-M2020-IM-P", "commonName": "M2020", "manufacturer": "Cesaroni Technology",
  "propellant": "Imax", "type": "reload", "delays": ["0"],
  "diameter": 0.075, "length": 0.757,
  "propellantMass": 4.349, "totalMass": 7.0318,
  "isp": 197.63,                        // optional: derived from impulse / propellant mass when absent
  "nozzleExitDiameter": 0.045,          // optional: enables ambient-pressure thrust correction
  "testPressure": 101325,               // ambient pressure of the static test
  "cgFromTop": 0.3785,                  // optional, default: half the length
  "massModel": "impulse-proportional",  // | "constant-isp" | "table"
  "thrustCurve": { "interpolation": "linear", "time": [0, 0.023, ...], "thrust": [0, 2070.11, ...] },
  "propellantMassCurve": { "time": [...], "mass": [...] },          // optional ("table" mass model)
  "source": { "url": "https://www.thrustcurve.org/simfiles/...", "license": "...", "contributor": "..." }
}
```

- **`interpolation`.** One of `step`, `linear` (the default; it reproduces the certified total impulse), `spline` (natural cubic),
  `akima` or `pchip` (monotone, no overshoot).
- **Missing values.** When a value is missing on load, the propellant mass is derived from the Isp (or the Isp from the propellant
  mass). If neither is known, an Isp of 200 s is assumed. The case mass is then derived from a 0.55 propellant mass fraction.
  `Motor::warnings()` and `rocketup motor <file>` list every assumption.
- **Overriding values.** Any value can be overridden when loading, with `MotorLoadOptions` in C++ or
  `--isp/--propellant/--total` on the command line.

## Rocket designs (`.rocketup`)

```json
{
  "format": "rocketup-rocket", "version": 1, "name": "Orbit Chaser", "designer": "...", "description": "...",
  "stack": [ <body component>, ... ],                 // nose to tail
  "separations": [ { "name": "Staging", "aftSectionStart": "Interstage", "trigger": {...},
                     "relativeSpeed": 2.0, "forwardName": "Sustainer", "aftName": "Booster" } ],
  "aerodynamics": { "dragSource": "computed", "cdMultiplier": 1.0, "plumeReducesBaseDrag": true,
                    "referenceDiameter": 0.13, "table": { "file": "cd.csv" } },
  "dryMassOverride": { "mass": 21.229, "cg": 0.886 }  // measured mass/CG without motor (optional)
}
```

Every component has `type`, `name` and optional `comment`, `massOverride` (kg), `cgOverride` (m from its front) and `children`.
Children have a `position`: `{"anchor": "top" | "middle" | "bottom", "offset": m}`.

| `type` | Fields |
|---|---|
| `nose-cone` | `shape` (conical, ogive, ellipsoid, power, parabolic, haack), `shapeParameter`, `length`, `baseRadius`, `thickness`, `filled`, `material`, `finish`, `shoulder {length, radius, thickness}` |
| `body-tube` | `length`, `radius`, `thickness`, `filled`, `material`, `finish` |
| `transition` | `length`, `foreRadius`, `aftRadius`, `shape`, `shapeParameter`, `thickness`, `material`, `finish` |
| `fin-set` | `count`, `outline` [[x, y], ...] (root leading edge at 0,0; or `"trapezoidal": {rootChord, tipChord, span, sweep}` / `"elliptical": {rootChord, span}`), `thickness`, `crossSection` (square, rounded, airfoil), `cant`, `rotation`, `material`, `finish`, `tab {height, length, position}` |
| `launch-lug` | `length`, `outerRadius`, `thickness`, `material` |
| `rail-button` | `count`, `diameter`, `height`, `massEach` |
| `inner-tube` | `length`, `outerRadius` (number or `"auto"`), `thickness`, `material` |
| `ring` | `role` (bulkhead, centering-ring, engine-block), `length`, `outerRadius` (`"auto"`), `innerRadius`, `material` |
| `mass` | `mass`, `length`, `radius`, `category` |
| `motor-mount` | inner-tube fields + `motor` (embedded motor object, a path string, or `{"file": ..., "isp": ..., ...}`), `overhang`, `ignition` (trigger) |
| `parachute` | `diameter`, `cd`, `spillHoleDiameter`, `canopyMaterial`, `lines {count, length, material}`, `deploy` (trigger), `deployDelay`, `inflationTime`, `packed {length, radius}` |
| `streamer` | `length`, `width`, `cd`, `material`, `deploy`, `deployDelay`, `inflationTime` |
| `payload` | mass fields + `eject` (trigger), `ejectionSpeed`, `freeFallDragArea`; children: parachutes, streamers, masses |

`material` is a library name (`"aluminum"`, `"fiberglass"`, `"carbon fiber"`, `"PLA"`, `"plywood (birch)"`, `"ripstop nylon"`, ...)
or `{"name": ..., "density": ..., "kind": "bulk" | "surface" | "line"}`. `finish` is one of polished, smooth, regular, unfinished, rough.

### Triggers

```json
{"type": "launch"}                         {"type": "never"}
{"type": "time", "time": 12.5}             {"type": "apogee", "delay": 1.0}
{"type": "burnout", "delay": 0.5, "source": "Booster motor"}
{"type": "altitude-below", "altitude": 600}   {"type": "altitude-above", "altitude": 1000}
{"type": "speed-below", "speed": 30}
{"type": "event", "event": "separation", "delay": 1.0, "source": "Staging"}
{"type": "external", "channel": "drogue"}  // command from a flight computer (HIL) or GUI
{"type": "baro-apogee", "drop": 5}         {"type": "baro-altitude-below", "altitude": 600}
{"type": "all", "of": [ ... ]}  {"type": "any", "of": [ ... ]}  {"type": "not", "of": { ... }}
{"type": "custom", "name": "my function", "fallback": { ... }}   // C++ lambda registered by name
```

Any trigger may carry `"latchDelay": s`: it fires `s` seconds after its condition first became true.
Event names are launch, ignition, liftoff, rail-exit, burnout, apogee, deployment, separation, ejection and landing.

## Planets

```json
{
  "format": "rocketup-planet", "version": 1, "name": "Titan",
  "gravitationalParameter": 8.978e12,      // or "mass": kg
  "radius": 2574730.0, "rotationRate": 4.56e-6,
  "gravity": {"model": "constant", "value": 1.352},    // optional
  "atmosphere": {
    "type": "layered",                     // | "us1976" | "table" | "exponential" | "vacuum"
    "gas": { "molarMass": 0.0276, "gamma": 1.4, "sutherland": {"mu0": 1.663e-5, "T0": 273.15, "S": 107} },  // or "air" | "co2" | "n2"
    "surfaceTemperature": 93.7, "surfacePressure": 146700.0, "geopotential": true,
    "layers": [ {"baseAltitude": 0, "lapseRate": -0.00053}, {"baseAltitude": 44000, "lapseRate": 0} ],
    "topAltitude": 150000
  }
}
```

- **`table` atmosphere.** Takes `"file": "profile.csv"` with columns altitude, temperature, pressure and optionally density, or the
  same data as inline arrays.
- **`exponential` atmosphere.** Takes `surfacePressure`, `temperature` and `scaleHeight`.
- **Pressure.** Every model integrates pressure hydrostatically and derives density from the ideal gas law. The speed of sound
  comes from γRT and the viscosity from Sutherland's law.

## Environments and projects

```json
{
  "format": "rocketup-project", "version": 1, "name": "Orbit Chaser at TEKNOFEST",
  "rocket": "../rockets/orbit_chaser.rocketup",              // or an inline rocket object
  "environment": {
    "planet": "earth",                                       // builtin name, "file.json", {"file": ...} or an inline planet
    "site": {"name": "Aksaray", "latitude": 38.4, "longitude": 34.0, "altitude": 970},
    "conditions": {"temperature": 297.15, "pressure": 90200}, // measured at the pad (optional)
    "rail": {"length": 6, "elevation": 85, "azimuth": 0, "friction": 0},
    "wind": {"type": "power-law", "speed": 5, "direction": 270, "referenceHeight": 10, "turbulence": 0.1, "seed": 7},
    "rotatingFrame": false
  },
  "simulation": {"integrator": "rk4", "dynamics": "6dof", "timeStep": 0.01, "maxTime": 3600,
                 "stopAtApogee": false, "recordInterval": 0, "descentRecordInterval": 0.05,
                 "sensors": {"imuRate": 100, "baroNoise": 3, "accelNoise": 0.08}},
  "output": {"directory": "../../output/orbit_chaser"}
}
```

- **Wind types.** `none`, `constant {speed, direction}`, `power-law {speed, direction, referenceHeight, exponent}`,
  `table {altitude[], speed[], direction[]}` or `{file}`, and `turbulent {mean, intensity, seed}`.
  `direction` is where the wind blows **from**, in degrees clockwise from north.
- **Integrators.** `euler`, `midpoint`, `heun`, `rk4` and `rk45` (adaptive).
- **Dynamics.** `6dof` or `3dof`.

## Simulation output

`io::exportAll` writes the following files:

- **`flight_<body>.csv`.** One file per body (rocket, separated sections, payloads) with about 50 columns:
  time, phase, altitude AGL/ASL, east/north, latitude/longitude, total/vertical/horizontal velocity, airspeed, acceleration
  (total, vertical, axial as an accelerometer would read it), Mach, Reynolds, dynamic pressure, angle of attack, thrust, drag, normal force,
  gravity, Cd (total/friction/pressure/base), CNα, CP, CG, stability margin, mass, propellant mass, inertias, roll/pitch/yaw rates,
  zenith/azimuth, wind, air temperature/pressure/density, speed of sound, recovery drag area and force. Units are written in the header.
- **`events.csv`.** Every event with time, body, source, altitude and speed.
- **`summary.json`.** Key figures, events, and per-body recovery (deployment conditions, peak opening load) and landing data
  (position, distance, bearing, impact speed, descent rate, kinetic energy, drift from apogee).
- **`trajectory.kml`.** 3-D trajectories, events and landing points for Google Earth.
- **`report.html`.** The interactive report. It loads Plotly from a CDN; set `ReportOptions::plotlyUrl` for offline use.

The column list is also available at run time through `recordColumns()`, with key, label, unit and accessor.
