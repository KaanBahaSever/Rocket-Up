# Validation

Every number on this page is reproduced by the test suite (`ctest`), by `examples/teknofest_3dof`, or by the snippets mentioned below.

## 1. OpenRocket: Orbit Chaser (TEKNOFEST 2021 mid-altitude rocket)

The reference is the team's OpenRocket 15.03 model of the same rocket, with the simulation data stored inside the `.ork` file:

- 130 mm fiberglass airframe, 2.025 m long, power-series nose (n = 0.5, 0.12 m), four aluminium free-form fins;
- launch site at 970 m (Aksaray), 6 m rail tilted 5° to the north, 6 m/s wind from the south;
- OpenRocket keeps the full base drag during the burn, so `plumeReducesBaseDrag = false` is used for the comparison.

| Cesaroni M2020 (8429 Ns) | OpenRocket | Rocket-Up 6-DOF RK4 | Δ |
|---|---:|---:|---:|
| Apogee | 3111.6 m | 3109.1 m | −0.08 % |
| Time to apogee | 25.39 s | 25.37 s | −0.07 % |
| Max velocity | 263.46 m/s | 263.47 m/s | 0.00 % |
| Max Mach | 0.7879 | 0.7879 | 0.00 % |
| Max acceleration | 85.68 m/s² | 85.86 m/s² | +0.2 % |
| Rail exit velocity | 31.95 m/s | 31.97 m/s | +0.1 % |

| Cesaroni M2150 (7443 Ns) | OpenRocket | Rocket-Up | Δ |
|---|---:|---:|---:|
| Apogee | 2985.5 m | 2982.0 m | −0.12 % |
| Time to apogee | 24.61 s | 24.62 s | +0.04 % |
| Max velocity | 272.08 m/s | 272.10 m/s | 0.00 % |
| Max Mach | 0.8128 | 0.8130 | +0.02 % |
| Rail exit velocity | 32.59 m/s | 32.60 m/s | 0.0 % |

The two trajectories also agree point by point (M2020):

| t (s) | altitude OR / RU (m) | velocity OR / RU (m/s) | Cd OR / RU |
|---:|---:|---:|---:|
| 1.86 | 141.5 / 141.8 | 155.17 / 155.35 | 0.497 / 0.500 |
| 3.71 | 550.7 / 551.7 | 263.33 / 263.36 | 0.551 / 0.554 |
| 9.30 | 1777.5 / 1777.8 | 175.83 / 175.63 | 0.506 / 0.509 |
| 14.09 | 2471.8 / 2471.2 | 116.25 / 116.03 | 0.485 / 0.488 |
| 20.09 | 2973.5 / 2971.8 | 52.66 / 52.46 | 0.472 / 0.475 |
| 24.89 | 3110.4 / 3107.9 | 7.26 / 6.95 | - |

Drag build-up at Mach 0.787, compared term by term with OpenRocket:

| | friction | pressure | base | total |
|---|---:|---:|---:|---:|
| OpenRocket | 0.2489 | 0.1014 | 0.2006 | 0.5509 |
| Rocket-Up | 0.254 | 0.100 | 0.201 | 0.554 |

The wind direction matters. With the same wind blowing across the rail instead of against it, both programs lose about 1 % of
apogee to weathercocking.

## 2. Analytic and reference cases

| Test | Expected | Result |
|---|---|---|
| Vacuum flight, constant g: apogee vs burnout energy `h_bo + v_bo²/2g` | exact | < 0.2 % |
| Vacuum burnout speed vs rocket equation `Isp g0 ln(m0/m1) − g t_b` | exact | < 0.5 % |
| US Standard Atmosphere 1976 at 0, 1, 5, 11, 20 and 32 km (T, p, ρ) | table | < 0.2 % |
| Mars surface density (NASA Glenn: 0.699 kPa, −31 °C) | 0.0150 kg/m³ | 0.0153 kg/m³ |
| Barrowman nose CNα | 2.0 | 2.0 |
| Cone / tangent ogive CP | 0.667 L / 0.466 L | exact / 0.466 L |
| Barrowman fin CNα and CP (trapezoidal, 4 fins, incompressible) | closed form | < 0.2 % / < 1.5 mm |
| Flat-plate turbulent Cf at Re 2.3·10⁶ | 0.00373 | 0.00373 |
| Motor files: `.eng`, `.rse`, ThrustCurve `.csv`, `.rumotor` of the same motor | 8428.7 Ns (ThrustCurve) | 8428.7 Ns, all formats |
| ∫ ṁ dt over the burn | propellant mass | < 0.05 % |

## 3. Integrators

M2020, 6 m/s wind, flown to apogee (time is wall clock on a laptop):

| Integrator | dt = 0.05 s | dt = 0.01 s | dt = 0.001 s |
|---|---:|---:|---:|
| Euler | 848 m (unstable) | 2559 m (unstable) | 3108.8 m (124 ms) |
| Midpoint | 3115.4 m | 3109.0 m | 3109.1 m |
| Heun | 3091.1 m | 3109.1 m | 3109.1 m |
| RK4 | 3114.1 m (5 ms) | **3109.1 m (24 ms)** | 3109.1 m (239 ms) |
| RK45 adaptive | - | 3109.0 m (514 steps, 8 ms) | - |

Explicit Euler is unstable for the rotational dynamics of a stable rocket at 10 ms steps. The restoring moment acts like a stiff
spring, with ω ≈ 10 rad/s at max q. Euler only converges with ~1 ms steps, ten times more work than RK4 for the same answer.
For 3-DOF point-mass runs, which have no rotation, Euler at 10 ms is within 0.1 % of RK4. That is why the old Python
simulator gave sensible results with Euler.

## 4. TEKNOFEST 2021 3-DOF data set and the old Python simulator

`examples/teknofest_3dof` uses the competition data set: 25 kg, 4.659 kg propellant, Isp 209.5 s, 85° rail, Cd(Mach, altitude) table,
Euler at 10 ms, constant gravity.

| | apogee | t_apogee | v_max | Mach max | range |
|---|---:|---:|---:|---:|---:|
| Rocket-Up, 3-DOF Euler 10 ms (rules) | 4524 m | 29.4 s | 360.8 m/s | 1.069 | 793 m |
| Rocket-Up, 3-DOF RK4 | 4524 m | 29.4 s | 360.7 m/s | 1.068 | 795 m |
| Rocket-Up, 6-DOF RK4 | 4527 m | 29.4 s | 360.7 m/s | 1.068 | 777 m |
| Old Python simulator (2021) | 4385 m | 29.0 s | 352 m/s (vertical) | 1.06 | 1391 m |

The differences come from simplifications in the old code:

- **No launch rail.** The old code started the rocket at 0.5 m/s, and the first thrust sample is 0 N, so gravity bent the path
  before the rocket gained speed. Removing the rail alone in Rocket-Up (`teknofest_3dof` with a 5 cm rail) moves the range
  from 793 m to 976 m and lowers the apogee by 24 m.
- **Cd lookup.** Cd was looked up with `if Mach < 0.1 ... elif Mach < 0.2 ...` steps instead of being interpolated in Mach and altitude.
- **Position update.** The position was advanced with the already-updated velocity plus ½·a·dt², so the acceleration was counted twice.
- **Propellant consumption.** It was not clamped: 4.68 kg were burned out of the 4.659 kg loaded.
