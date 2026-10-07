#pragma once

/// Rocket-Up: modular rocket flight simulation library.
/// Include this header to get the whole public API (including nlohmann::json for the
/// toJson()/fromJson() functions).

#include <nlohmann/json.hpp>

#include "rocketup/aero/AeroModel.hpp"
#include "rocketup/analysis/MonteCarlo.hpp"
#include "rocketup/components/BodyComponents.hpp"
#include "rocketup/components/Component.hpp"
#include "rocketup/components/FinSet.hpp"
#include "rocketup/components/InternalComponents.hpp"
#include "rocketup/components/Material.hpp"
#include "rocketup/components/Recovery.hpp"
#include "rocketup/core/Io.hpp"
#include "rocketup/core/Types.hpp"
#include "rocketup/environment/Atmosphere.hpp"
#include "rocketup/environment/Environment.hpp"
#include "rocketup/environment/Planet.hpp"
#include "rocketup/environment/Wind.hpp"
#include "rocketup/hil/HilBridge.hpp"
#include "rocketup/hil/SerialPort.hpp"
#include "rocketup/io/Export.hpp"
#include "rocketup/io/HtmlReport.hpp"
#include "rocketup/io/Project.hpp"
#include "rocketup/io/RocketDrawing.hpp"
#include "rocketup/propulsion/Motor.hpp"
#include "rocketup/rocket/Rocket.hpp"
#include "rocketup/sim/Events.hpp"
#include "rocketup/sim/FlightState.hpp"
#include "rocketup/sim/Results.hpp"
#include "rocketup/sim/Sensors.hpp"
#include "rocketup/sim/Simulation.hpp"
#include "rocketup/sim/Trigger.hpp"

#define ROCKETUP_VERSION_MAJOR 0
#define ROCKETUP_VERSION_MINOR 1
#define ROCKETUP_VERSION_PATCH 0
#define ROCKETUP_VERSION_STRING "0.1.0"
