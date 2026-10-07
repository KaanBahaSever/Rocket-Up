# Aerodynamic model

Rocket-Up computes the drag coefficient, the normal-force slope and the centre of pressure from the geometry alone. It does this
with semi-empirical methods that have been validated against wind tunnel data for decades, and needs no CFD. The model is
evaluated every integration step, so Cd follows Mach number, Reynolds number (altitude and speed), angle of attack and the motor state.

All coefficients use the reference area `A = π d²/4`, where `d` is the largest body diameter unless `aerodynamics.referenceDiameter` is set.

## 1. Normal force and centre of pressure (Barrowman)

| Part | CNα (1/rad) | CP |
|---|---|---|
| Nose cone / shoulder / boattail | `2 (A_aft − A_fore) / A` | `x0 + (L·A_aft − V) / (A_aft − A_fore)` (V = enclosed volume) |
| Body lift (Galejs) | `CN = 1.1 · A_plan / A · sin²α` | planform centroid |
| Fin set (subsonic) | `CNα₁ = 2π s²/A / (1 + √(1 + (β s² / (A_f cos Γc))²))` | MAC leading edge + MAC·0.25 |
| Fin set (supersonic, M ≥ 1.5) | `CNα₁ = 4 A_f / (A β)` | moves aft to `(ARβ − 0.67)/(2ARβ − 1)` of the MAC |

- **Fin sets.** The single-fin slope is multiplied by the fin-count factor (1, 1, 1.5, 2, 2.37, 2.74, 2.99, 3.24 for 1–8 fins)
  and by the body interference factor `1 + r/(s + r)`.
- **Transonic range.** Values between Mach 0.9 and 1.5 are interpolated.
- **Fin geometry.** Fins are arbitrary polygons. Mean aerodynamic chord, mid-chord sweep, leading-edge sweep and spanwise
  strips are integrated numerically, so trapezoidal, elliptical, clipped-delta and free-form fins all use the same code.
- **CP over Mach and AoA.** The total CP is the CN-weighted average of the component CPs. It therefore moves with Mach number
  and with angle of attack: body lift pulls it forward at high AoA.

## 2. Drag build-up (Niskanen / OpenRocket)

`Cd = (Cd_friction + Cd_pressure + Cd_base) × cdMultiplier`, then the axial coefficient at angle of attack is
`C_A = Cd · f(α)`, with f rising from 1 to 1.3 at 17° and falling to 0 at 90°.

### Skin friction

- Reynolds number: `Re = V·L/ν` with `ν` from Sutherland's law of the local gas.
- `Cf = 1/(1.50 ln Re − 5.6)²` (turbulent), `1.48e-2` below Re = 10⁴.
- Compressibility: `× (1 − 0.1 M²)` subsonic, `× (1 + 0.15 M²)^−0.58` supersonic, blended between M 0.9 and 1.1.
- Roughness limit: `Cf ≥ 0.032 (k/L)^0.2 × (compressibility)`, where the roughness height k comes from the surface finish of each
  component (polished 2 µm, smooth 20 µm, regular paint 60 µm, unfinished 150 µm, rough 500 µm).
- `Cd_f = [ Σ Cf·A_wet,body · (1 + 1/(2 f_B)) + Σ Cf·(1 + 2t/c̄)·A_wet,fins + Σ Cf·A_wet,lugs ] / A`, where `f_B` is the fineness ratio.

### Pressure drag

| Source | Model |
|---|---|
| Conical nose / shoulder | `0.8 sin²φ` subsonic; `sin φ` at M = 1; `2.1 sin²φ + 0.5 sinφ/√(M²−1)` above M 1.3; smooth Hermite blend between |
| Other nose shapes | Negligible below M 0.8. Above that, the cone value times a shape factor (ogive 0.70, Von Kármán 0.50, LV-Haack 0.55, power series and parabolic by parameter, ellipsoid 1.4) |
| Boattail | base drag × `(3 − γ)/2` for `1 < γ = L/(d_fore − d_aft) < 3` |
| Blunt front / steps | stagnation pressure `0.85 (1 + M²/4 + M⁴/40)` (subsonic) on the exposed annulus |
| Fins | leading edge: stagnation (square) or `(1 − M²)^−0.417 − 1` (rounded/airfoil) × cos²Γ_LE; trailing edge: base drag (square) or ½ base drag (rounded) |
| Launch lugs, rail buttons | stagnation pressure on the frontal area, plus skin friction on the wetted area |

### Base drag

`Cd_b = (0.12 + 0.13 M²)` subsonic, `0.25/M` supersonic, applied to the aft area. While a motor burns, its exhaust plume fills
part of the base. The motor's cross-section is then removed from the base area (`plumeReducesBaseDrag`, on by default; OpenRocket
does not do this). This is why Cd drops a little during the burn in the flight data.

## 3. Damping

- **Pitch and yaw.** OpenRocket's multiplier
  `0.275 d̄ (x_cg⁴ + (L − x_cg)⁴)/(A d) + Σ 0.6 min(N,4) A_f |x_fin − x_cg|³/(A d)`, applied ×3 as `½ ρ ω² A d C`.
- **Roll.** Forcing `N (y_MAC + r) CNα₁ δ / d` from fin cant δ. Damping from strip theory `(2π/β) N ω Σ c ξ² Δξ / (A d V)`.

## 4. Descent

Once a recovery device opens, or a section separates after apogee, the body is flown as a point mass.

- **Parachutes.** Drag area `Cd · π (D² − D_spill²)/4`. It grows quadratically during the inflation time, which gives the
  opening shock. The peak load of each device is reported.
- **Tumbling sections.** `0.6 × planform area + 0.6 × N A_fin`, roughly a cylinder in cross-flow averaged over random orientation.
- **Free-falling payloads.** Their own drag area, either the value you set or a tumbling cylinder estimate.

## 5. Using your own data

When wind-tunnel, RASAero or CFD data exist, they can replace the total drag. The stability model stays the same:

```json
"aerodynamics": {
  "dragSource": "table",
  "table": { "file": "../aero/my_rocket_cd.csv", "interpolation": "linear" },
  "cdMultiplier": 1.0
}
```

The CSV is either `mach,cd`, or `mach,<alt1>,<alt2>,...` with one Cd column per altitude in m ASL
(see `data/aero/teknofest2021_cd_vs_mach.csv`). A constant also works: `"dragSource": "constant", "constantCd": 0.45`.
`cdMultiplier` calibrates any source against flight data and is perturbed by the Monte Carlo analysis.

## 6. Accuracy and limits

- **Subsonic and low transonic flight.** Cd and CP match OpenRocket within 1 %, and the flight matches within 0.5 %
  (see [VALIDATION.md](VALIDATION.md)). Expect ±10 % against real flights, which is typical for every component build-up method.
- **Transonic and supersonic flight (M > 0.8).** The nose wave-drag shape factors are approximate (±20 %). Use a RASAero or CFD
  table for high-performance flights.
- **Not modelled.** Fin flutter, protuberances other than lugs and rail buttons, and Magnus forces.

## References

- J. S. Barrowman, J. A. Barrowman, *The Theoretical Prediction of the Center of Pressure*, NARAM-8, 1966
- J. S. Barrowman, *The Practical Calculation of the Aerodynamic Characteristics of Slender Finned Vehicles*, M.Sc. thesis, 1967
- S. Niskanen, *Development of an Open Source Model Rocket Simulation Software*, M.Sc. thesis, Helsinki University of Technology, 2009
- S. F. Hoerner, *Fluid-Dynamic Drag*, 1965
- G. M. Gregorek, *Aerodynamic Drag of Model Rockets*, Estes TR-11, 1970
