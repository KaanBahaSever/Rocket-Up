#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rocketup/core/Types.hpp"

namespace rocketup {

/// Thermodynamic state of the atmosphere at one point.
struct AtmosphereState {
    double temperature = 288.15;        ///< K
    double pressure = 101325.0;         ///< Pa
    double density = 1.225;             ///< kg/m^3
    double speedOfSound = 340.29;       ///< m/s
    double dynamicViscosity = 1.789e-5; ///< Pa s

    /// Kinematic viscosity (m^2/s); infinite in vacuum.
    double kinematicViscosity() const;
};

/// Ideal-gas properties of an atmosphere's working fluid.
struct GasProperties {
    double molarMass = 0.0289644;  ///< kg/mol
    double gamma = 1.4;            ///< ratio of specific heats
    // Sutherland's law: mu = mu0 (T/T0)^1.5 (T0 + S) / (T + S)
    double sutherlandMu0 = 1.716e-5;  ///< Pa s
    double sutherlandT0 = 273.15;     ///< K
    double sutherlandS = 110.4;       ///< K

    double specificGasConstant() const;  ///< J/(kg K)
    double viscosity(double temperature) const;
    double speedOfSound(double temperature) const;
    double density(double pressure, double temperature) const;

    json toJson() const;
    static GasProperties fromJson(const json& j);
    static GasProperties air();
    static GasProperties carbonDioxide();
    static GasProperties nitrogen();
};

/// Interface of every atmosphere model. Altitudes are geometric, above the planet datum (m).
class AtmosphereModel {
public:
    virtual ~AtmosphereModel() = default;
    virtual AtmosphereState at(double altitude) const = 0;
    virtual std::string type() const = 0;
    virtual json toJson() const = 0;
    virtual std::unique_ptr<AtmosphereModel> clone() const = 0;

    /// Build any atmosphere model from JSON (`{"type": "layered" | "table" | "exponential" | "vacuum", ...}`).
    /// `planetRadius` and `surfaceGravity` are used by hydrostatic models.
    static std::unique_ptr<AtmosphereModel> fromJson(const json& j, double planetRadius, double surfaceGravity,
                                                     const std::string& baseDirectory = "");
};

/// Piecewise-linear temperature profile integrated hydrostatically (US Standard Atmosphere 1976 style).
/// Works for any planet: give the gas, the surface state and the lapse-rate layers.
class LayeredAtmosphere final : public AtmosphereModel {
public:
    struct Layer {
        double baseAltitude = 0.0;  ///< m (geopotential if geopotential altitudes are enabled)
        double lapseRate = 0.0;     ///< K/m (negative = cooling with height)
    };

    LayeredAtmosphere(GasProperties gas, double baseTemperature, double basePressure, std::vector<Layer> layers,
                      double referenceGravity, double planetRadius, bool useGeopotential = true,
                      double topAltitude = 1e30);

    AtmosphereState at(double altitude) const override;
    std::string type() const override { return "layered"; }
    json toJson() const override;
    std::unique_ptr<AtmosphereModel> clone() const override { return std::make_unique<LayeredAtmosphere>(*this); }

    /// Offsets the whole temperature profile (e.g. ISA+10). Surface pressure is kept.
    void setTemperatureOffset(double deltaKelvin);
    double temperatureOffset() const { return temperatureOffset_; }

    /// Calibrate the profile so that the given temperature/pressure are met at `altitude`
    /// (use with measured launch-site weather).
    void setConditionsAt(double altitude, double temperature, double pressure);

    const GasProperties& gas() const { return gas_; }
    const std::vector<Layer>& layers() const { return layers_; }
    double baseTemperature() const { return baseTemperature_; }
    double basePressure() const { return basePressure_; }

    /// US Standard Atmosphere 1976 (0 - 86 km).
    static LayeredAtmosphere usStandard1976(double planetRadius = 6356766.0);

private:
    GasProperties gas_;
    double baseTemperature_;
    double basePressure_;
    std::vector<Layer> layers_;
    double g0_;
    double radius_;
    bool geopotential_;
    double top_;
    double temperatureOffset_ = 0.0;
    std::vector<double> layerT_, layerP_;  // state at each layer base (cached)

    void rebuild();
    double toModelAltitude(double z) const;
    void stateAtModelAltitude(double h, double& T, double& p) const;
};

/// Tabulated atmosphere (altitude, temperature, pressure[, density]) e.g. from a radiosonde or GCM.
class TableAtmosphere final : public AtmosphereModel {
public:
    TableAtmosphere(GasProperties gas, std::vector<double> altitude, std::vector<double> temperature,
                    std::vector<double> pressure, std::vector<double> density = {});
    /// CSV columns: altitude_m, temperature_K, pressure_Pa[, density_kgm3]
    static TableAtmosphere fromCsv(const std::string& path, GasProperties gas);

    AtmosphereState at(double altitude) const override;
    std::string type() const override { return "table"; }
    json toJson() const override;
    std::unique_ptr<AtmosphereModel> clone() const override { return std::make_unique<TableAtmosphere>(*this); }

private:
    GasProperties gas_;
    std::vector<double> h_, T_, p_, rho_;
    mathrix::Interpolator1D temperature_, logPressure_, density_;
};

/// Isothermal exponential atmosphere: p = p0 exp(-h / H).
class ExponentialAtmosphere final : public AtmosphereModel {
public:
    ExponentialAtmosphere(GasProperties gas, double surfacePressure, double temperature, double scaleHeight);
    AtmosphereState at(double altitude) const override;
    std::string type() const override { return "exponential"; }
    json toJson() const override;
    std::unique_ptr<AtmosphereModel> clone() const override { return std::make_unique<ExponentialAtmosphere>(*this); }

private:
    GasProperties gas_;
    double p0_, T_, H_;
};

/// No atmosphere at all (Moon, asteroids, ...).
class VacuumAtmosphere final : public AtmosphereModel {
public:
    AtmosphereState at(double altitude) const override;
    std::string type() const override { return "vacuum"; }
    json toJson() const override;
    std::unique_ptr<AtmosphereModel> clone() const override { return std::make_unique<VacuumAtmosphere>(*this); }
};

}  // namespace rocketup
