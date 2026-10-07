# Hardware in the loop

Rocket-Up can act as the outside world for a real flight computer. While the simulation runs, usually in real time, it sends
the board the sensor values the board would measure in flight. The board answers with commands, and those commands fire
parachutes, separations or ignitions in the simulation. You can test the deployment logic on a desk, with realistic noise,
before it flies.

```
 Rocket-Up                                         flight computer (Arduino, ESP32, STM32, ...)
 ─────────                                         ───────────────
 6-DOF flight ──► SensorSimulator ──► $RUSEN ... ──► your firmware
                                                      │ launch / apogee / main logic
 Trigger::external("drogue") ◄────── $RUCMD,drogue ◄──┘
```

## Running it

1. Flash `examples/arduino/RocketUpHIL/RocketUpHIL.ino`, or your own firmware.
2. Use a design whose recovery devices listen to external channels, for example `data/rockets/orbit_chaser_hil.rocketup`
   (`"deploy": {"type": "external", "channel": "drogue"}`).
3. Run the command below. `rocketup ports` lists the available ports.

```bash
rocketup hil data/projects/orbit_chaser_hil.project.json --port COM3 --baud 115200 --rate 50
```

- `--speed 2` runs twice as fast as real time.
- `hil_flight_computer` with no arguments runs the same flight against an emulated flight computer inside the process. This is useful for
  developing the algorithm before porting it to the board.

In C++:

```cpp
auto link = std::make_shared<hil::SerialLink>("COM3", 115200);          // or hil::LoopbackLink(device function)
auto bridge = std::make_shared<hil::HilBridge>(link, hil::HilOptions{});
Simulation sim(rocket, env);
sim.addObserver(bridge);
SimulationResult r = sim.run();
for (auto& [t, channel] : bridge->receivedCommands()) std::cout << t << " " << channel << "\n";
```

## Protocol

The protocol is plain text, one message per line (`\n`), framed like NMEA: `$BODY*CS`. `CS` is two hex digits, the XOR of all
characters between `$` and `*`. A board may omit `*CS`, but a wrong checksum is rejected.

| Direction | Message | Fields |
|---|---|---|
| sim → board | `$RUHELLO,<rocket>` | sent once when the simulation starts (reset your state) |
| sim → board | `$RUSEN,t,p,T,ax,ay,az,gx,gy,gz,h,fix,lat,lon,alt` | see below, sent at `--rate` Hz |
| sim → board | `$RUEVT,t,event,source` | true events (launch, burnout, apogee, ...), for logging and scoring |
| sim → board | `$RUEND` | flight finished |
| board → sim | `$RUCMD,<channel>` | fires `Trigger::external("<channel>")` (also accepted: `DEPLOY <channel>`) |
| board → sim | `$RULOG,<text>` | printed by the simulator and stored in `HilBridge::deviceLog()` |

`$RUSEN` fields:

| Field | Unit | Meaning |
|---|---|---|
| t | ms | simulation time since launch |
| p | Pa | static pressure (barometer, with noise) |
| T | °C | air temperature |
| ax, ay, az | m/s² | specific force in the body frame (x towards the nose); reads about +9.8 on x on the pad |
| gx, gy, gz | rad/s | body angular rates (gyroscope) |
| h | m | barometric altitude above the pad, from the standard-atmosphere formula |
| fix | 0/1 | GPS fix |
| lat, lon | deg | GPS position (with noise) |
| alt | m | GPS altitude above sea level |

Sensor rates, noise, ranges and saturation are set in `SimulationOptions::sensors` (`imuRate`, `baroNoise`, `accelNoise`,
`accelRange`, `gyroNoise`, `gpsRate`, ...).

## Tips

- Many boards reset when the port opens. `HilOptions::startupDelay` (2 s) waits before the first message.
- At 115200 baud a `$RUSEN` line takes about 10 ms. Keep `--rate` at 50 Hz or less on 8-bit boards.
- Triggers can combine sensor logic with external commands as a safety net, for example
  `Trigger::external("drogue") || Trigger::apogee(3.0)` deploys a backup 3 s after the true apogee.
