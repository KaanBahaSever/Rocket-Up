#include "rocketup/environment/Wind.hpp"

#include <cmath>
#include <nlohmann/json.hpp>
#include <random>

#include "rocketup/core/Io.hpp"

namespace rocketup {

using mathrix::deg2rad;

Vec3 WindModel::vectorFrom(double speed, double fromDirectionDeg) {
    const double a = deg2rad(fromDirectionDeg);
    // Blowing FROM azimuth a means moving TOWARDS a + 180 deg.
    return {-speed * std::sin(a), -speed * std::cos(a), 0.0};
}

std::unique_ptr<WindModel> WindModel::fromJson(const json& j, const std::string& baseDirectory) {
    if (j.is_null()) return std::make_unique<NoWind>();
    const std::string type = j.value("type", std::string("none"));
    std::unique_ptr<WindModel> w;
    if (type == "none" || type == "calm") w = std::make_unique<NoWind>();
    else if (type == "constant")
        w = std::make_unique<ConstantWind>(j.at("speed").get<double>(), j.value("direction", 0.0));
    else if (type == "power-law" || type == "powerlaw")
        w = std::make_unique<PowerLawWind>(j.at("speed").get<double>(), j.value("direction", 0.0),
                                           j.value("referenceHeight", 10.0), j.value("exponent", 1.0 / 7.0));
    else if (type == "table") {
        if (j.contains("file")) w = std::make_unique<TableWind>(
                                    TableWind::fromCsv(io::resolvePath(j.at("file").get<std::string>(), baseDirectory)));
        else
            w = std::make_unique<TableWind>(j.at("altitude").get<std::vector<double>>(),
                                            j.at("speed").get<std::vector<double>>(),
                                            j.at("direction").get<std::vector<double>>());
    } else if (type == "transformed") {
        const auto& o = j.at("offset");
        return std::make_unique<TransformedWind>(fromJson(j.at("base"), baseDirectory), j.value("scale", 1.0),
                                                 j.value("rotation", 0.0),
                                                 Vec3{o.at(0).get<double>(), o.at(1).get<double>(), o.at(2).get<double>()});
    } else if (type == "turbulent") {
        return std::make_unique<TurbulentWind>(fromJson(j.at("mean"), baseDirectory), j.value("intensity", 0.1),
                                               j.value("seed", static_cast<uint64_t>(1)), j.value("minimumSigma", 0.0));
    } else
        throw RocketUpError("unknown wind type '" + type + "'");
    // Shorthand: {"type":"constant", ..., "turbulence": 0.1}
    if (j.contains("turbulence") && j.at("turbulence").get<double>() > 0.0)
        return std::make_unique<TurbulentWind>(std::move(w), j.at("turbulence").get<double>(),
                                               j.value("seed", static_cast<uint64_t>(1)));
    return w;
}

json NoWind::toJson() const { return {{"type", "none"}}; }

ConstantWind::ConstantWind(double speed, double fromDirectionDeg)
    : speed_(speed), direction_(fromDirectionDeg), v_(vectorFrom(speed, fromDirectionDeg)) {}

json ConstantWind::toJson() const { return {{"type", "constant"}, {"speed", speed_}, {"direction", direction_}}; }

PowerLawWind::PowerLawWind(double referenceSpeed, double fromDirectionDeg, double referenceHeight, double exponent)
    : vref_(referenceSpeed), dir_(fromDirectionDeg), href_(referenceHeight), alpha_(exponent) {
    if (href_ <= 0.0) throw RocketUpError("power-law wind reference height must be positive");
}

Vec3 PowerLawWind::velocity(double altitude, double) const {
    const double h = std::max(altitude, 0.1);
    return vectorFrom(vref_ * std::pow(h / href_, alpha_), dir_);
}

json PowerLawWind::toJson() const {
    return {{"type", "power-law"},
            {"speed", vref_},
            {"direction", dir_},
            {"referenceHeight", href_},
            {"exponent", alpha_}};
}

TableWind::TableWind(std::vector<double> altitude, std::vector<double> speed, std::vector<double> fromDirectionDeg)
    : h_(std::move(altitude)), s_(std::move(speed)), d_(std::move(fromDirectionDeg)) {
    if (h_.empty() || h_.size() != s_.size() || h_.size() != d_.size())
        throw RocketUpError("wind table columns must be non-empty and of equal length");
    std::vector<double> e(h_.size()), n(h_.size());
    for (size_t i = 0; i < h_.size(); ++i) {
        const Vec3 v = vectorFrom(s_[i], d_[i]);
        e[i] = v.x;
        n[i] = v.y;
    }
    east_ = mathrix::Interpolator1D(h_, e, mathrix::Interp::Linear, mathrix::Extrapolation::Clamp);
    north_ = mathrix::Interpolator1D(h_, n, mathrix::Interp::Linear, mathrix::Extrapolation::Clamp);
}

TableWind TableWind::fromCsv(const std::string& path) {
    const auto t = io::readCsvFile(path);
    if (t.columnCount() < 3) throw RocketUpError("wind CSV '" + path + "' needs altitude, speed, direction columns");
    return TableWind(t.column(0), t.column(1), t.column(2));
}

Vec3 TableWind::velocity(double altitude, double) const { return {east_(altitude), north_(altitude), 0.0}; }

json TableWind::toJson() const { return {{"type", "table"}, {"altitude", h_}, {"speed", s_}, {"direction", d_}}; }

TurbulentWind::TurbulentWind(std::unique_ptr<WindModel> mean, double intensity, uint64_t seed, double minimumSigma)
    : mean_(std::move(mean)), intensity_(intensity), minSigma_(minimumSigma), seed_(seed) {
    if (!mean_) mean_ = std::make_unique<NoWind>();
    build();
}

TurbulentWind::TurbulentWind(const TurbulentWind& o)
    : mean_(o.mean_->clone()), intensity_(o.intensity_), minSigma_(o.minSigma_), seed_(o.seed_), modes_(o.modes_) {}

void TurbulentWind::build() {
    std::mt19937_64 rng(seed_);
    std::uniform_real_distribution<double> phase(0.0, mathrix::kTwoPi);
    constexpr int kModes = 24;
    const double f0 = 0.01, f1 = 3.0;  // Hz
    double sumA2 = 0.0;
    modes_.clear();
    for (int i = 0; i < kModes; ++i) {
        const double f = f0 * std::pow(f1 / f0, (i + 0.5) / kModes);
        const double df = f * std::log(f1 / f0) / kModes;
        // von Karman-like longitudinal spectrum shape, length/speed scale folded into fc.
        const double fc = 0.08;
        const double S = 1.0 / std::pow(1.0 + 70.8 * (f / fc) * (f / fc), 5.0 / 6.0);
        const double a = std::sqrt(S * df);
        modes_.push_back({f, a, phase(rng), phase(rng), phase(rng)});
        sumA2 += 0.5 * a * a;
    }
    const double norm = 1.0 / std::sqrt(sumA2);  // unit variance
    for (auto& m : modes_) m.amplitude *= norm;
}

Vec3 TurbulentWind::velocity(double altitude, double time) const {
    const Vec3 mean = mean_->velocity(altitude, time);
    const double speed = mean.norm();
    const double sigma = std::max(intensity_ * speed, minSigma_);
    if (sigma <= 0.0) return mean;
    double u = 0.0, v = 0.0, w = 0.0;
    for (const auto& m : modes_) {
        const double arg = mathrix::kTwoPi * m.frequency * time;
        u += m.amplitude * std::cos(arg + m.phaseU);
        v += m.amplitude * std::cos(arg + m.phaseV);
        w += m.amplitude * std::cos(arg + m.phaseW);
    }
    // Longitudinal gusts along the mean wind, weaker lateral/vertical components.
    const Vec3 along = speed > 1e-6 ? mean / speed : Vec3{0, 1, 0};
    const Vec3 lateral = Vec3{0, 0, 1}.cross(along).normalized();
    return mean + along * (sigma * u) + lateral * (0.75 * sigma * v) + Vec3{0, 0, 0.5 * sigma * w};
}

json TurbulentWind::toJson() const {
    json j = {{"type", "turbulent"}, {"mean", mean_->toJson()}, {"intensity", intensity_}, {"seed", seed_}};
    if (minSigma_ > 0.0) j["minimumSigma"] = minSigma_;
    return j;
}

TransformedWind::TransformedWind(std::unique_ptr<WindModel> base, double scale, double rotationDeg, Vec3 offset)
    : base_(base ? std::move(base) : std::make_unique<NoWind>()), scale_(scale), rotation_(rotationDeg), offset_(offset) {}

TransformedWind::TransformedWind(const TransformedWind& o)
    : base_(o.base_->clone()), scale_(o.scale_), rotation_(o.rotation_), offset_(o.offset_) {}

Vec3 TransformedWind::velocity(double altitude, double time) const {
    const Vec3 v = base_->velocity(altitude, time) * scale_;
    // Rotating the "from" direction clockwise by r rotates the vector clockwise too.
    const double a = -deg2rad(rotation_);
    return Vec3{v.x * std::cos(a) - v.y * std::sin(a), v.x * std::sin(a) + v.y * std::cos(a), v.z} + offset_;
}

json TransformedWind::toJson() const {
    return {{"type", "transformed"},
            {"base", base_->toJson()},
            {"scale", scale_},
            {"rotation", rotation_},
            {"offset", {offset_.x, offset_.y, offset_.z}}};
}

}  // namespace rocketup
