#pragma once

#include <optional>
#include <string>
#include <vector>

#include "rocketup/core/Types.hpp"

namespace rocketup {

/// How propellant mass is consumed during the burn.
enum class MassFlowModel {
    /// Mass decreases in proportion to delivered impulse: m(t) = m0 (1 - I(t)/Itot).
    /// Always burns exactly the stated propellant mass (OpenRocket's model). Default.
    ImpulseProportional,
    /// mdot = F / (Isp g0) with a fixed Isp (e.g. TEKNOFEST rules). Mass may not reach zero
    /// at burnout if Isp and propellant mass are inconsistent.
    ConstantIsp,
    /// Use the tabulated propellant-mass curve shipped with the motor (RockSim .rse files).
    Table,
};

const char* toString(MassFlowModel m);
MassFlowModel massFlowModelFromString(const std::string& s);

/// Where the data came from (kept for attribution in reports).
struct MotorSource {
    std::string url;
    std::string contributor;
    std::string license;
    std::string importedFrom;  ///< original file name / format
    std::string notes;
};

/// A solid (or any thrust-curve based) rocket motor.
///
/// Thrust curves found online are often incomplete (no Isp, no masses, unsorted points).
/// `finalize()` fills in what can be derived and records a warning for everything assumed:
///   - propellant mass  <- total impulse / (Isp g0)
///   - Isp              <- total impulse / (propellant mass g0)
///   - neither known    <- Isp assumed 200 s (typical APCP), warning
/// Ambient-pressure correction: F = F_test + (p_test - p_ambient) A_exit, when the nozzle
/// exit area is known (important for high altitude or other planets).
class Motor {
public:
    Motor() = default;

    // ---- identification
    std::string designation;   ///< e.g. "8429M2020-P"
    std::string commonName;    ///< e.g. "M2020"
    std::string manufacturer;  ///< e.g. "Cesaroni Technology"
    std::string propellant;    ///< e.g. "Imax"
    std::string motorType = "reload";  ///< reload | single-use | hybrid | liquid
    std::vector<std::string> delays;
    MotorSource source;

    // ---- geometry & mass (SI)
    double diameter = 0.0;        ///< m
    double length = 0.0;          ///< m
    double propellantMass = 0.0;  ///< kg (0 = unknown)
    double totalMass = 0.0;       ///< loaded motor mass, kg (0 = unknown)
    double isp = 0.0;             ///< s (0 = unknown / derive)
    double nozzleExitArea = 0.0;  ///< m^2 (0 = unknown -> no pressure correction)
    double testPressure = constants::kSeaLevelPressure;  ///< ambient pressure during the static test
    double cgFromTop = -1.0;      ///< m from the forward end (-1 = geometric centre)

    // ---- performance data
    std::vector<double> time;    ///< s
    std::vector<double> thrust;  ///< N
    mathrix::Interp interpolation = mathrix::Interp::Linear;  ///< linear reproduces certified impulse
    std::vector<double> massTime;        ///< optional propellant-mass table, s
    std::vector<double> massPropellant;  ///< optional propellant-mass table, kg
    MassFlowModel massModel = MassFlowModel::ImpulseProportional;
    double thrustScale = 1.0;  ///< multiplies the whole curve (for dispersion studies)

    /// Sorts data, adds the (0,0) start point, derives missing values. Must be called after
    /// editing any field above (loaders call it for you).
    void finalize();

    /// Choose how thrust is interpolated between the data points and rebuild the curve:
    /// Interp::Step, Linear (default, reproduces the certified impulse), CubicSpline,
    /// Akima or Pchip (smooth and monotone, never overshoots).
    Motor& setInterpolation(Interp method);
    /// Choose how propellant mass is consumed and rebuild the mass model.
    Motor& setMassModel(MassFlowModel model);
    const std::vector<std::string>& warnings() const { return warnings_; }

    // ---- derived quantities (valid after finalize)
    double burnTime() const { return burnTime_; }
    double totalImpulse() const { return totalImpulse_ * thrustScale; }
    double averageThrust() const { return burnTime_ > 0 ? totalImpulse() / burnTime_ : 0.0; }
    double maxThrust() const { return maxThrust_ * thrustScale; }
    double effectiveIsp() const;  ///< Itot / (mprop g0)
    double dryMass() const { return totalMass - propellantMass; }
    std::string impulseClass() const;  ///< "M", "K", ... (NAR/TRA letter)
    std::string displayName() const;

    /// Thrust (N) at `t` seconds after ignition and the given ambient pressure (Pa).
    double thrustAt(double t, double ambientPressure = constants::kSeaLevelPressure) const;
    /// Impulse delivered between ignition and `t` (N s, at test pressure).
    double impulseAt(double t) const;
    double propellantMassAt(double t) const;
    double massAt(double t) const { return dryMass() + propellantMassAt(t); }
    double massFlowAt(double t) const;
    /// Axial CG position of the loaded motor measured from its forward end (m).
    double cgAt(double t) const;

    json toJson() const;
    static Motor fromJson(const json& j);

private:
    std::vector<std::string> warnings_;
    mathrix::Interpolator1D curve_;
    mathrix::Interpolator1D massCurve_;
    double burnTime_ = 0.0;
    double totalImpulse_ = 0.0;
    double maxThrust_ = 0.0;
};

/// Values used to complete partially specified motor files.
struct MotorLoadOptions {
    std::optional<std::string> designation;
    std::optional<std::string> manufacturer;
    std::optional<double> propellantMass;  ///< kg
    std::optional<double> totalMass;       ///< kg
    std::optional<double> isp;             ///< s
    std::optional<double> diameter;        ///< m
    std::optional<double> length;          ///< m
    std::optional<double> nozzleExitDiameter;  ///< m
    std::optional<mathrix::Interp> interpolation;
    std::optional<MassFlowModel> massModel;
};

/// Reads every common thrust-curve format:
///   .eng (RASP), .rse (RockSim XML), .csv/.txt (ThrustCurve.org CSV or plain time,thrust),
///   .rumotor / .json (RocketUp motor format, see docs/FILE_FORMATS.md).
class MotorLoader {
public:
    static Motor load(const std::string& path, const MotorLoadOptions& options = {});
    /// Files can contain several motors (.eng and .rse).
    static std::vector<Motor> loadAll(const std::string& path, const MotorLoadOptions& options = {});

    static std::vector<Motor> parseEng(const std::string& text);
    static std::vector<Motor> parseRse(const std::string& text);
    static Motor parseCsv(const std::string& text);

    /// Save in the RocketUp motor format (.rumotor, JSON).
    static void save(const Motor& motor, const std::string& path);
};

}  // namespace rocketup
