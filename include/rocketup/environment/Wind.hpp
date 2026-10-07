#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rocketup/core/Types.hpp"

namespace rocketup {

/// Wind field. Velocities are in the local East-North-Up frame (m/s); `altitude` is above
/// ground level (launch site). Directions follow the meteorological convention: the
/// direction the wind blows FROM, in degrees clockwise from north.
class WindModel {
public:
    virtual ~WindModel() = default;
    virtual Vec3 velocity(double altitude, double time) const = 0;
    virtual std::string type() const = 0;
    virtual json toJson() const = 0;
    virtual std::unique_ptr<WindModel> clone() const = 0;

    static std::unique_ptr<WindModel> fromJson(const json& j, const std::string& baseDirectory = "");
    /// Helper: ENU vector of a wind of `speed` blowing from `fromDirectionDeg`.
    static Vec3 vectorFrom(double speed, double fromDirectionDeg);
};

class NoWind final : public WindModel {
public:
    Vec3 velocity(double, double) const override { return {}; }
    std::string type() const override { return "none"; }
    json toJson() const override;
    std::unique_ptr<WindModel> clone() const override { return std::make_unique<NoWind>(); }
};

class ConstantWind final : public WindModel {
public:
    ConstantWind(double speed, double fromDirectionDeg);
    Vec3 velocity(double, double) const override { return v_; }
    std::string type() const override { return "constant"; }
    json toJson() const override;
    std::unique_ptr<WindModel> clone() const override { return std::make_unique<ConstantWind>(*this); }

private:
    double speed_, direction_;
    Vec3 v_;
};

/// Boundary-layer profile: V(h) = Vref (h / href)^alpha (alpha ~ 1/7 over open terrain).
class PowerLawWind final : public WindModel {
public:
    PowerLawWind(double referenceSpeed, double fromDirectionDeg, double referenceHeight = 10.0,
                 double exponent = 1.0 / 7.0);
    Vec3 velocity(double altitude, double) const override;
    std::string type() const override { return "power-law"; }
    json toJson() const override;
    std::unique_ptr<WindModel> clone() const override { return std::make_unique<PowerLawWind>(*this); }

private:
    double vref_, dir_, href_, alpha_;
};

/// Wind profile table (altitude AGL, speed, from-direction). Linear interpolation of the
/// east/north components, clamped outside the table.
class TableWind final : public WindModel {
public:
    TableWind(std::vector<double> altitude, std::vector<double> speed, std::vector<double> fromDirectionDeg);
    /// CSV columns: altitude_m, speed_mps, direction_deg
    static TableWind fromCsv(const std::string& path);
    Vec3 velocity(double altitude, double) const override;
    std::string type() const override { return "table"; }
    json toJson() const override;
    std::unique_ptr<WindModel> clone() const override { return std::make_unique<TableWind>(*this); }

private:
    std::vector<double> h_, s_, d_;
    mathrix::Interpolator1D east_, north_;
};

/// Adds deterministic, seeded gusts on top of any mean wind. The gust signal is a sum of
/// sinusoids following a von Karman-like spectrum (stateless in time, so it is safe to
/// evaluate at arbitrary integrator stages).
class TurbulentWind final : public WindModel {
public:
    /// `intensity`: gust standard deviation as a fraction of the local mean speed (OpenRocket's
    /// "turbulence intensity"). `minimumSigma`: gust floor in m/s for calm conditions.
    TurbulentWind(std::unique_ptr<WindModel> mean, double intensity, uint64_t seed = 1, double minimumSigma = 0.0);
    TurbulentWind(const TurbulentWind& o);
    Vec3 velocity(double altitude, double time) const override;
    std::string type() const override { return "turbulent"; }
    json toJson() const override;
    std::unique_ptr<WindModel> clone() const override { return std::make_unique<TurbulentWind>(*this); }
    const WindModel& mean() const { return *mean_; }

private:
    std::unique_ptr<WindModel> mean_;
    double intensity_, minSigma_;
    uint64_t seed_;
    struct Mode {
        double frequency, amplitude, phaseU, phaseV, phaseW;
    };
    std::vector<Mode> modes_;
    void build();
};

/// Any wind model scaled, rotated about the vertical and shifted by a constant vector
/// (used by Monte Carlo dispersion).
class TransformedWind final : public WindModel {
public:
    TransformedWind(std::unique_ptr<WindModel> base, double scale, double rotationDeg, Vec3 offset);
    TransformedWind(const TransformedWind& o);
    Vec3 velocity(double altitude, double time) const override;
    std::string type() const override { return "transformed"; }
    json toJson() const override;
    std::unique_ptr<WindModel> clone() const override { return std::make_unique<TransformedWind>(*this); }

private:
    std::unique_ptr<WindModel> base_;
    double scale_, rotation_;
    Vec3 offset_;
};

}  // namespace rocketup
