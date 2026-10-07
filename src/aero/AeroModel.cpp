#include "rocketup/aero/AeroModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocketup {

using mathrix::kPi;
using mathrix::smoothstep;
using mathrix::sq;

double AeroCoefficients::totalDrag(double aoa) const { return axial * std::cos(aoa) + normal * std::sin(aoa); }

// ------------------------------------------------------------------ correlations

double AeroModel::stagnationCd(double m) {
    double pressure;
    if (m <= 1.0) pressure = 1.0 + m * m / 4.0 + std::pow(m, 4) / 40.0;
    else pressure = 1.84 - 0.76 / (m * m) + 0.166 / std::pow(m, 4) + 0.035 / std::pow(m, 6);
    return 0.85 * pressure;
}

double AeroModel::baseCd(double m) { return m <= 1.0 ? 0.12 + 0.13 * m * m : 0.25 / m; }

double AeroModel::skinFrictionCoefficient(double re, double m, double roughness, double length) {
    // Fully turbulent smooth-wall value (Niskanen eq. 3.79 - 3.81).
    double cf = re < 1e4 ? 1.48e-2 : 1.0 / sq(1.50 * std::log(re) - 5.6);
    const double c1 = 1.0 - 0.1 * m * m;
    const double c2 = 1.0 / std::pow(1.0 + 0.15 * m * m, 0.58);
    if (m < 0.9) cf *= c1;
    else if (m < 1.1) cf *= c2 * (m - 0.9) / 0.2 + c1 * (1.1 - m) / 0.2;
    else cf *= c2;
    // Roughness limited value.
    if (roughness > 0.0 && length > 0.0) {
        double rc;
        if (m < 0.9) rc = 1.0 - 0.1 * m * m;
        else if (m > 1.1) rc = 1.0 / (1.0 + 0.18 * m * m);
        else {
            const double a = 1.0 - 0.1 * sq(0.9), b = 1.0 / (1.0 + 0.18 * sq(1.1));
            rc = b * (m - 0.9) / 0.2 + a * (1.1 - m) / 0.2;
        }
        cf = std::max(cf, 0.032 * std::pow(roughness / length, 0.2) * rc);
    }
    return cf;
}

namespace {

/// Axial force multiplier vs angle of attack (OpenRocket's fit: 1 -> 1.3 at 17 deg -> 0 at 90 deg).
double axialMultiplier(double aoa) {
    aoa = mathrix::clamp(aoa, 0.0, kPi);
    const bool reversed = aoa > 0.5 * kPi;
    if (reversed) aoa = kPi - aoa;
    const double a17 = mathrix::deg2rad(17.0);
    double m;
    if (aoa < a17) {
        const double t = aoa / a17;
        m = 1.0 + 0.3 * (3 * t * t - 2 * t * t * t);
    } else {
        const double u = (aoa - a17) / (0.5 * kPi - a17);
        m = 1.3 * (1.0 - (3 * u * u - 2 * u * u * u));
    }
    return reversed ? -m : m;
}

/// Wave/pressure drag of a cone with half angle phi (based on its base area).
double conePressureCd(double sinphi, double m) {
    const double cd0 = 0.8 * sinphi * sinphi;
    double cd;
    if (m <= 0.8) cd = cd0;
    else if (m <= 1.0) cd = cd0 + (sinphi - cd0) * smoothstep(0.8, 1.0, m);
    else {
        auto sup = [sinphi](double mm) { return 2.1 * sinphi * sinphi + 0.5 * sinphi / std::sqrt(mm * mm - 1.0); };
        if (m <= 1.3) {
            // Cubic Hermite blend: secant slope at M = 1, slope of the supersonic fit at M = 1.3.
            const double y0 = sinphi, y1 = sup(1.3), h = 0.3;
            const double d0 = (y1 - y0) / h;
            const double d1 = -0.5 * sinphi * 1.3 / std::pow(1.3 * 1.3 - 1.0, 1.5);
            const double t = (m - 1.0) / h, t2 = t * t, t3 = t2 * t;
            cd = (2 * t3 - 3 * t2 + 1) * y0 + (t3 - 2 * t2 + t) * h * d0 + (-2 * t3 + 3 * t2) * y1 + (t3 - t2) * h * d1;
        } else {
            cd = sup(m);
        }
    }
    return std::min(cd, AeroModel::stagnationCd(m));
}

/// Supersonic wave drag of a shape relative to a cone of equal fineness (Hoerner/Stoney trends).
double shapeFactor(NoseShape shape, double p) {
    switch (shape) {
        case NoseShape::Conical: return 1.0;
        case NoseShape::Ogive: return 0.70;
        case NoseShape::Haack: return 0.50 + 0.15 * mathrix::clamp(p / 0.3333, 0.0, 1.0);
        case NoseShape::Ellipsoid: return 1.40;
        case NoseShape::PowerSeries: {
            static const mathrix::Interpolator1D f({0.0, 0.25, 0.5, 0.75, 1.0}, {1.6, 0.9, 0.62, 0.55, 1.0},
                                                  mathrix::Interp::Pchip);
            return f(mathrix::clamp(p, 0.0, 1.0));
        }
        case NoseShape::Parabolic: {
            static const mathrix::Interpolator1D f({0.0, 0.5, 0.75, 1.0}, {1.0, 0.75, 0.65, 0.60}, mathrix::Interp::Pchip);
            return f(mathrix::clamp(p, 0.0, 1.0));
        }
    }
    return 1.0;
}

}  // namespace

// ------------------------------------------------------------------ construction

AeroModel::AeroModel(const std::vector<PlacedComponent>& parts, const AeroSettings& settings, double referenceDiameter)
    : settings_(settings), refDiameter_(referenceDiameter), refArea_(kPi * referenceDiameter * referenceDiameter / 4.0) {
    build(parts);
}

AeroModel::AeroModel(const Rocket& rocket) : AeroModel(rocket.flatten(), rocket.aero(), rocket.referenceDiameter()) {}

void AeroModel::build(const std::vector<PlacedComponent>& parts) {
    for (const auto& p : parts) {
        const Component* c = p.component;
        if (auto* s = dynamic_cast<const SymmetricComponent*>(c)) {
            Body b;
            b.name = s->name();
            b.type = s->type();
            b.x0 = p.position;
            b.length = s->length();
            b.rFore = s->foreRadius();
            b.rAft = s->aftRadius();
            b.wettedArea = s->wettedArea();
            b.planformArea = s->planformArea();
            b.planformCentroid = s->planformCentroid();
            b.volume = s->volume();
            b.roughness = roughnessHeight(s->finish());
            if (auto* n = dynamic_cast<const NoseCone*>(s)) {
                b.isNose = true;
                b.shape = n->shape();
                b.shapeParameter = n->shapeParameter();
                hasNose_ = true;
            } else if (auto* t = dynamic_cast<const Transition*>(s)) {
                b.shape = t->shape();
                b.shapeParameter = t->shapeParameter();
            }
            if (b.length > 0.0) bodies_.push_back(b);
        } else if (auto* f = dynamic_cast<const FinSet*>(c)) {
            Fins fs;
            fs.name = f->name();
            fs.x0 = p.position;
            fs.g = f->geometry();
            fs.count = f->count();
            fs.thickness = f->thickness();
            fs.cant = f->cantAngle();
            fs.bodyRadius = f->bodyRadius();
            fs.roughness = roughnessHeight(f->finish());
            fs.cross = f->crossSection();
            if (fs.g.area > 0.0) fins_.push_back(fs);
        } else if (auto* l = dynamic_cast<const LaunchLug*>(c)) {
            protrusions_.push_back({l->name(), l->type(), p.position, l->frontalArea(), l->wettedArea(), roughnessHeight(l->finish())});
        } else if (auto* r = dynamic_cast<const RailButton*>(c)) {
            protrusions_.push_back({r->name(), r->type(), p.position, r->frontalArea(),
                                    r->count() * kPi * r->length() * r->height(), roughnessHeight(SurfaceFinish::Regular)});
        }
    }
    std::sort(bodies_.begin(), bodies_.end(), [](const Body& a, const Body& b) { return a.x0 < b.x0; });
    if (!bodies_.empty()) {
        front_ = bodies_.front().x0;
        length_ = bodies_.back().x0 + bodies_.back().length - front_;
        double plan = 0.0, len = 0.0;
        for (const auto& b : bodies_) {
            maxRadius_ = std::max({maxRadius_, b.rFore, b.rAft});
            plan += b.planformArea;
            len += b.length;
        }
        avgDiameter_ = len > 0.0 ? plan / len : 0.0;
        tumbleCdA_ = 0.6 * plan;
    }
    for (const auto& f : fins_) tumbleCdA_ += 0.6 * f.count * f.g.area;
    if (!settings_.tableMach.empty() && !settings_.tableCd.empty()) {
        std::vector<double> alts = settings_.tableAltitude;
        if (alts.empty()) alts = {0.0};
        if (alts.size() != settings_.tableCd.size())
            throw RocketUpError("Cd table: number of altitude rows does not match");
        cdTable_ = mathrix::Table2D(settings_.tableMach, alts, settings_.tableCd, settings_.tableInterpolation);
    }
    if (refDiameter_ <= 0.0) {
        refDiameter_ = std::max(2.0 * maxRadius_, 1e-3);
        refArea_ = kPi * refDiameter_ * refDiameter_ / 4.0;
    }
}

// ------------------------------------------------------------------ fins

double AeroModel::finSetFactor(int n) {
    static const double table[] = {0.0, 1.0, 1.0, 1.5, 2.0, 2.37, 2.74, 2.99, 3.24};
    if (n <= 8) return table[n];
    return 0.81 * n / 2.0;
}

double AeroModel::finCnAlphaSingle(const Fins& f, double m) const {
    const double s = f.g.span, A = f.g.area;
    auto subsonic = [&](double mm) {
        const double beta = std::sqrt(std::max(1e-6, 1.0 - mm * mm));
        const double cosG = std::max(0.05, std::cos(f.g.midChordSweep));
        return 2.0 * kPi * s * s / refArea_ / (1.0 + std::sqrt(1.0 + sq(beta * s * s / (A * cosG))));
    };
    auto supersonic = [&](double mm) { return 4.0 * A / (refArea_ * std::sqrt(mm * mm - 1.0)); };
    if (m <= 0.9) return subsonic(m);
    if (m >= 1.5) return supersonic(m);
    const double a = subsonic(0.9), b = supersonic(1.5);
    return a + (b - a) * (m - 0.9) / 0.6;
}

double AeroModel::finCpFraction(const Fins& f, double m) const {
    if (m <= 0.5) return 0.25;
    const double ar = std::max(f.g.aspectRatio, 0.1);
    const double beta2 = std::sqrt(3.0);  // sqrt(2^2 - 1)
    const double at2 = mathrix::clamp((ar * beta2 - 0.67) / (2.0 * ar * beta2 - 1.0), 0.25, 0.5);
    if (m >= 2.0) {
        const double beta = std::sqrt(m * m - 1.0);
        return mathrix::clamp((ar * beta - 0.67) / (2.0 * ar * beta - 1.0), 0.25, 0.5);
    }
    return 0.25 + (at2 - 0.25) * (m - 0.5) / 1.5;
}

// ------------------------------------------------------------------ drag

double AeroModel::nosePressureCd(const Body& b, double m) const {
    const double h = b.rAft - b.rFore;
    const double sinphi = h / std::sqrt(h * h + b.length * b.length);
    const double cone = conePressureCd(sinphi, m);
    if (b.shape == NoseShape::Conical) return cone;
    return shapeFactor(b.shape, b.shapeParameter) * cone * smoothstep(0.8, 1.05, m);
}

void AeroModel::drag(const FlightConditions& c, double& friction, double& pressure, double& base,
                     std::vector<ComponentAero>* detail) const {
    friction = pressure = base = 0.0;
    const double m = c.mach;
    const double nu = std::isfinite(c.kinematicViscosity) && c.kinematicViscosity > 0 ? c.kinematicViscosity : 1e30;
    const double re = c.airspeed * length_ / nu;
    const double fB = maxRadius_ > 0.0 ? length_ / (2.0 * maxRadius_) : 10.0;
    const double bodyForm = 1.0 + 1.0 / (2.0 * std::max(fB, 1.0));
    auto add = [&](const std::string& name, const std::string& type, double f, double p, double bse) {
        friction += f;
        pressure += p;
        base += bse;
        if (detail) detail->push_back({name, type, 0.0, 0.0, f, p, bse});
    };

    for (size_t i = 0; i < bodies_.size(); ++i) {
        const Body& b = bodies_[i];
        const double cf = skinFrictionCoefficient(re, m, b.roughness, length_);
        const double f = cf * bodyForm * b.wettedArea / refArea_;
        double p = 0.0, bs = 0.0;
        const double aFore = kPi * b.rFore * b.rFore, aAft = kPi * b.rAft * b.rAft;
        // Blunt front face (e.g. aft section after separation).
        if (i == 0 && b.rFore > 1e-4) p += stagnationCd(m) * aFore / refArea_;
        // Step from the previous component.
        if (i > 0) {
            const double prevAft = kPi * sq(bodies_[i - 1].rAft);
            if (aFore > prevAft + 1e-9) p += stagnationCd(m) * (aFore - prevAft) / refArea_;
            else if (prevAft > aFore + 1e-9) bs += baseCd(m) * (prevAft - aFore) / refArea_;
        }
        if (b.rAft > b.rFore + 1e-6) {
            p += nosePressureCd(b, m) * (aAft - aFore) / refArea_;
        } else if (b.rAft < b.rFore - 1e-6) {
            const double gamma = b.length / (2.0 * (b.rFore - b.rAft));
            double mul = 0.0;
            if (gamma < 1.0) mul = 1.0;
            else if (gamma < 3.0) mul = (3.0 - gamma) / 2.0;
            p += mul * baseCd(m) * (aFore - aAft) / refArea_;
        }
        if (i + 1 == bodies_.size()) {
            const double area = std::max(0.0, aAft - c.plumeArea);
            bs += baseCd(m) * area / refArea_;
        }
        add(b.name, b.type, f, p, bs);
    }

    for (const auto& fs : fins_) {
        const double cf = skinFrictionCoefficient(re, m, fs.roughness, length_);
        const double mac = std::max(fs.g.macLength, 1e-6);
        const double f = cf * (1.0 + 2.0 * fs.thickness / mac) * 2.0 * fs.count * fs.g.area / refArea_;
        double le;
        if (fs.cross == FinCrossSection::Square) {
            le = stagnationCd(m);
        } else {
            if (m < 0.9) le = std::pow(1.0 - m * m, -0.417) - 1.0;
            else if (m < 1.0) le = 1.0 - 1.785 * (m - 0.9);
            else le = 1.214 - 0.502 / (m * m) + 0.1095 / std::pow(m, 4);
        }
        double cd = le * fs.g.leadingEdgeCos2;
        if (fs.cross == FinCrossSection::Square) cd += baseCd(m);
        else if (fs.cross == FinCrossSection::Rounded) cd += 0.5 * baseCd(m);
        const double p = cd * fs.g.span * fs.thickness * fs.count / refArea_;
        add(fs.name, "fin-set", f, p, 0.0);
    }

    for (const auto& pr : protrusions_) {
        const double cf = skinFrictionCoefficient(re, m, pr.roughness, length_);
        add(pr.name, pr.type, cf * pr.wettedArea / refArea_, stagnationCd(m) * pr.frontalArea / refArea_, 0.0);
    }
}

// ------------------------------------------------------------------ compute

AeroCoefficients AeroModel::compute(const FlightConditions& c, double cg) const {
    AeroCoefficients r;
    r.referenceArea = refArea_;
    r.referenceLength = refDiameter_;
    const double m = c.mach;
    const double nu = std::isfinite(c.kinematicViscosity) && c.kinematicViscosity > 0 ? c.kinematicViscosity : 1e30;
    r.reynolds = c.airspeed * length_ / nu;

    drag(c, r.cdFriction, r.cdPressure, r.cdBase, nullptr);
    double cd0 = r.cdFriction + r.cdPressure + r.cdBase;
    switch (settings_.dragSource) {
        case DragSource::Computed: break;
        case DragSource::Table:
            if (!cdTable_.empty()) cd0 = cdTable_(m, c.altitudeAsl);
            break;
        case DragSource::Constant: cd0 = settings_.constantCd; break;
    }
    r.cd = cd0 * settings_.cdMultiplier;
    const double aoa = c.angleOfAttack;
    r.axial = r.cd * axialMultiplier(aoa);

    // ---- normal force & centre of pressure
    const double sa = std::sin(aoa);
    double cnaSum = 0.0, cnaMoment = 0.0, cnSum = 0.0, cnMoment = 0.0;
    for (const auto& b : bodies_) {
        const double aFore = kPi * b.rFore * b.rFore, aAft = kPi * b.rAft * b.rAft;
        const double dA = aAft - aFore;
        if (std::abs(dA) > 1e-12) {
            const double cna = 2.0 * dA / refArea_;
            const double x = b.x0 + (b.length * aAft - b.volume) / dA;
            cnaSum += cna;
            cnaMoment += cna * x;
            cnSum += cna * sa;
            cnMoment += cna * sa * x;
        }
        // Body lift (Galejs): significant at larger angles of attack.
        const double cnl = 1.1 * b.planformArea / refArea_ * sa * sa;
        cnSum += cnl;
        cnMoment += cnl * (b.x0 + b.planformCentroid);
    }
    double rollSumForcing = 0.0, rollSumDamping = 0.0;
    const double beta = std::sqrt(std::max(0.09, std::abs(1.0 - m * m)));
    for (const auto& f : fins_) {
        const double cna1 = finCnAlphaSingle(f, m);
        const double interference = 1.0 + f.bodyRadius / (f.g.span + f.bodyRadius);
        const double cna = cna1 * finSetFactor(f.count) * interference;
        const double x = f.x0 + f.g.macLeadingX + f.g.macLength * finCpFraction(f, m);
        cnaSum += cna;
        cnaMoment += cna * x;
        cnSum += cna * sa;
        cnMoment += cna * sa * x;
        if (f.cant != 0.0)
            rollSumForcing += f.count * (f.g.macSpanY + f.bodyRadius) * cna1 * f.cant / refDiameter_;
        double sum = 0.0;
        const double dy = f.g.stripY.size() > 1 ? f.g.stripY[1] - f.g.stripY[0] : f.g.span;
        for (size_t k = 0; k < f.g.stripY.size(); ++k)
            sum += f.g.stripChord[k] * sq(f.bodyRadius + f.g.stripY[k]) * dy;
        const double slope = m < 1.0 ? 2.0 * kPi / beta : 4.0 / beta;
        rollSumDamping += slope * f.count * sum;
    }
    r.cnAlpha = cnaSum;
    r.normal = cnSum;
    if (aoa > 1e-4 && std::abs(cnSum) > 1e-9) r.cp = cnMoment / cnSum;
    else if (std::abs(cnaSum) > 1e-12) r.cp = cnaMoment / cnaSum;
    else r.cp = front_ + 0.5 * length_;

    // ---- damping
    double mul = 0.275 * avgDiameter_ / (refArea_ * refDiameter_) *
                 (std::pow(std::abs(cg - front_), 4) + std::pow(std::abs(front_ + length_ - cg), 4));
    for (const auto& f : fins_) {
        const double xmid = f.x0 + f.g.macLeadingX + 0.5 * f.g.macLength;
        mul += 0.6 * std::min(f.count, 4) * f.g.area * std::pow(std::abs(xmid - cg), 3) / (refArea_ * refDiameter_);
    }
    r.pitchDamping = 3.0 * mul;
    r.rollForcing = rollSumForcing;
    const double v = std::max(c.airspeed, 1.0);
    r.rollDamping = rollSumDamping * c.rollRate / (refArea_ * refDiameter_ * v);
    return r;
}

std::vector<ComponentAero> AeroModel::breakdown(const FlightConditions& c) const {
    std::vector<ComponentAero> out;
    double f, p, b;
    drag(c, f, p, b, &out);
    const double m = c.mach;
    for (auto& ca : out) {
        for (const auto& body : bodies_) {
            if (body.name != ca.name) continue;
            const double aFore = kPi * body.rFore * body.rFore, aAft = kPi * body.rAft * body.rAft;
            const double dA = aAft - aFore;
            if (std::abs(dA) > 1e-12) {
                ca.cnAlpha = 2.0 * dA / refArea_;
                ca.cp = body.x0 + (body.length * aAft - body.volume) / dA;
            } else {
                ca.cp = body.x0 + 0.5 * body.length;
            }
        }
        for (const auto& pr : protrusions_)
            if (pr.name == ca.name) ca.cp = pr.x;
        for (const auto& fs : fins_) {
            if (fs.name != ca.name) continue;
            ca.cnAlpha = finCnAlphaSingle(fs, m) * finSetFactor(fs.count) * (1.0 + fs.bodyRadius / (fs.g.span + fs.bodyRadius));
            ca.cp = fs.x0 + fs.g.macLeadingX + fs.g.macLength * finCpFraction(fs, m);
        }
    }
    return out;
}

double AeroModel::stabilityMargin(double cg, double mach) const {
    FlightConditions c;
    c.mach = mach;
    c.airspeed = 100.0;
    const AeroCoefficients a = compute(c, cg);
    return (a.cp - cg) / refDiameter_;
}

}  // namespace rocketup
