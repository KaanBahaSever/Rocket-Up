#pragma once

#include <nlohmann/json_fwd.hpp>
#include <mathrix/mathrix.hpp>

#include <stdexcept>
#include <string>

namespace rocketup {

using mathrix::Extrapolation;
using mathrix::Integrator;
using mathrix::Interp;
using mathrix::Mat3;
using mathrix::Quat;
using mathrix::Vec3;
using json = nlohmann::json;

/// Physical constants (SI).
namespace constants {
inline constexpr double kStandardGravity = 9.80665;      ///< m/s^2, used for Isp conversions
inline constexpr double kUniversalGasConstant = 8.314462618;  ///< J/(mol K)
inline constexpr double kGravitationalConstant = 6.67430e-11;  ///< m^3/(kg s^2)
inline constexpr double kSeaLevelPressure = 101325.0;    ///< Pa
}  // namespace constants

/// Error thrown for invalid user input (bad files, inconsistent designs, ...).
class RocketUpError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

}  // namespace rocketup
