# Data

| Folder | Content |
|---|---|
| `motors/*.rumotor` | Motors in the RocketUp format, ready to use |
| `motors/source/` | The original files they were made from: RockSim `.rse` and RASP `.eng` from ThrustCurve.org, a ThrustCurve CSV export, and the TEKNOFEST 2021 validation thrust profile |
| `planets/` | Earth, Mars and Moon (identical to the builtins) plus Venus and Titan examples. Copy one to define your own world |
| `aero/` | Drag tables: `teknofest2021_cd_vs_mach.csv` is Cd(Mach) at 0, 3000, 6000 and 12000 m from the TEKNOFEST 2021 data set |
| `rockets/` | Rocket designs: `orbit_chaser.rocketup` and `orbit_chaser_hil.rocketup` (deployment commanded by an external flight computer) |
| `projects/` | Complete projects for `rocketup run` / `rocketup hil` |

## Motor data sources

| File | Source | License / note |
|---|---|---|
| Cesaroni 8429M2020-P | [ThrustCurve.org simfile 5f4294d20002e90000000747](https://www.thrustcurve.org/simfiles/5f4294d20002e90000000747/) (certification data) | public domain |
| Cesaroni 7455M2150-P | [ThrustCurve.org simfile 5f4294d20002e900000006b9](https://www.thrustcurve.org/simfiles/5f4294d20002e900000006b9/) (certification data) | no explicit license on ThrustCurve.org |

The RockSim files were chosen over the RASP files of the same motors because they also contain the propellant mass curve.
Add more motors with `python tools/thrustcurve_fetch.py <name> --convert build/bin/rocketup`.
