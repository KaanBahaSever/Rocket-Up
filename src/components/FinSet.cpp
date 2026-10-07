#include "rocketup/components/FinSet.hpp"

#include <algorithm>
#include <cmath>

#include "JsonHelpers.hpp"
#include "rocketup/components/BodyComponents.hpp"
#include "rocketup/core/Io.hpp"

namespace rocketup {

using mathrix::kPi;

const char* toString(FinCrossSection c) {
    switch (c) {
        case FinCrossSection::Square: return "square";
        case FinCrossSection::Rounded: return "rounded";
        case FinCrossSection::Airfoil: return "airfoil";
    }
    return "square";
}

FinCrossSection finCrossSectionFromString(const std::string& s) {
    const std::string n = io::toLower(s);
    if (n == "square" || n == "flat") return FinCrossSection::Square;
    if (n == "rounded" || n == "round") return FinCrossSection::Rounded;
    if (n == "airfoil") return FinCrossSection::Airfoil;
    throw RocketUpError("unknown fin cross-section '" + s + "'");
}

// ------------------------------------------------------------------ FinSet

FinSet::FinSet(std::string name, int count, std::vector<Point> outline, double thickness)
    : Component(std::move(name)), count_(count), thickness_(thickness) {
    if (outline.empty()) outline = {{0.0, 0.0}, {0.05, 0.05}, {0.1, 0.05}, {0.1, 0.0}};
    setOutline(std::move(outline));
    setCount(count);
    setPosition(Anchor::Bottom, 0.0);
}

FinSet FinSet::trapezoidal(std::string name, int count, double rootChord, double tipChord, double span,
                           double sweepLength, double thickness) {
    std::vector<Point> pts = {{0.0, 0.0}, {sweepLength, span}, {sweepLength + tipChord, span}, {rootChord, 0.0}};
    if (tipChord <= 0.0) pts = {{0.0, 0.0}, {sweepLength, span}, {rootChord, 0.0}};
    return FinSet(std::move(name), count, pts, thickness);
}

FinSet FinSet::elliptical(std::string name, int count, double rootChord, double span, double thickness) {
    std::vector<Point> le, te;
    constexpr int n = 24;
    for (int i = 0; i <= n; ++i) {
        const double y = span * std::sin(0.5 * kPi * i / n);
        const double c = rootChord * std::sqrt(std::max(0.0, 1.0 - (y / span) * (y / span)));
        le.push_back({0.5 * (rootChord - c), y});
        te.push_back({0.5 * (rootChord + c), y});
    }
    std::vector<Point> pts = le;
    for (int i = n - 1; i >= 0; --i) pts.push_back(te[static_cast<size_t>(i)]);
    pts.front() = {0.0, 0.0};
    pts.back() = {rootChord, 0.0};
    return FinSet(std::move(name), count, pts, thickness);
}

FinSet FinSet::freeform(std::string name, int count, std::vector<Point> outline, double thickness) {
    return FinSet(std::move(name), count, std::move(outline), thickness);
}

FinSet& FinSet::setCount(int n) {
    if (n < 1 || n > 32) throw RocketUpError("fin count must be between 1 and 32");
    count_ = n;
    return *this;
}

FinSet& FinSet::setOutline(std::vector<Point> pts) {
    if (pts.size() < 3) throw RocketUpError("fin outline needs at least 3 points");
    if (std::abs(pts.front().y) > 1e-9 || std::abs(pts.back().y) > 1e-9)
        throw RocketUpError("fin outline must start and end on the root (y = 0)");
    for (auto& p : pts)
        if (p.y < -1e-9) throw RocketUpError("fin outline points must have y >= 0");
    // Shift so that the root leading edge is at x = 0.
    const double x0 = std::min(pts.front().x, pts.back().x);
    for (auto& p : pts) p.x -= x0;
    outline_ = std::move(pts);
    return *this;
}

FinSet& FinSet::setTab(double height, double length, double fromRootLeadingEdge) {
    tabHeight_ = height;
    tabLength_ = length;
    tabPosition_ = fromRootLeadingEdge;
    return *this;
}

double FinSet::length() const { return std::abs(outline_.back().x - outline_.front().x); }

double FinSet::outerRadius() const {
    double s = 0.0;
    for (auto& p : outline_) s = std::max(s, p.y);
    return bodyRadius() + s;
}

double FinSet::bodyRadius() const {
    const Component* p = parent();
    if (!p) return 0.0;
    if (auto* s = dynamic_cast<const SymmetricComponent*>(p))
        return s->radiusAt(mathrix::clamp(positionInParent() + 0.5 * length(), 0.0, s->length()));
    return p->outerRadius();
}

double FinSet::planformArea() const {
    double a = 0.0;
    for (size_t i = 0; i < outline_.size(); ++i) {
        const auto& p = outline_[i];
        const auto& q = outline_[(i + 1) % outline_.size()];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * std::abs(a);
}

FinGeometry FinSet::geometry() const {
    FinGeometry g;
    g.rootChord = length();
    for (auto& p : outline_) g.span = std::max(g.span, p.y);
    g.area = planformArea();
    // Polygon centroid.
    double a2 = 0.0, cx = 0.0, cy = 0.0;
    for (size_t i = 0; i < outline_.size(); ++i) {
        const auto& p = outline_[i];
        const auto& q = outline_[(i + 1) % outline_.size()];
        const double cr = p.x * q.y - q.x * p.y;
        a2 += cr;
        cx += (p.x + q.x) * cr;
        cy += (p.y + q.y) * cr;
    }
    if (std::abs(a2) > 1e-15) {
        g.centroidX = cx / (3.0 * a2);
        g.centroidY = cy / (3.0 * a2);
    }
    // Tip chord: horizontal edge at full span.
    for (size_t i = 0; i < outline_.size(); ++i) {
        const auto& p = outline_[i];
        const auto& q = outline_[(i + 1) % outline_.size()];
        if (std::abs(p.y - g.span) < 1e-9 && std::abs(q.y - g.span) < 1e-9) g.tipChord = std::abs(q.x - p.x);
    }
    if (g.span <= 0.0) return g;
    // Spanwise strips.
    constexpr int n = 60;
    const double dy = g.span / n;
    double sumC = 0.0, sumC2 = 0.0, sumXle = 0.0, sumY = 0.0;
    for (int k = 0; k < n; ++k) {
        const double y = (k + 0.5) * dy;
        double xmin = 1e300, xmax = -1e300;
        for (size_t i = 0; i < outline_.size(); ++i) {
            const auto& p = outline_[i];
            const auto& q = outline_[(i + 1) % outline_.size()];
            if ((y - p.y) * (y - q.y) > 0.0 || std::abs(q.y - p.y) < 1e-15) continue;
            const double x = p.x + (y - p.y) * (q.x - p.x) / (q.y - p.y);
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
        }
        const double c = xmax > xmin ? xmax - xmin : 0.0;
        if (c <= 0.0) xmin = xmax = 0.0;
        g.stripY.push_back(y);
        g.stripChord.push_back(c);
        g.stripLeadingX.push_back(xmin);
        sumC += c * dy;
        sumC2 += c * c * dy;
        sumXle += xmin * c * dy;
        sumY += y * c * dy;
    }
    if (sumC > 0.0) {
        g.macLength = sumC2 / sumC;
        g.macLeadingX = sumXle / sumC;
        g.macSpanY = sumY / sumC;
    }
    double cos2 = 0.0;
    for (int k = 0; k + 1 < n; ++k) {
        const double dx = g.stripLeadingX[static_cast<size_t>(k + 1)] - g.stripLeadingX[static_cast<size_t>(k)];
        cos2 += dy * dy / (dy * dy + dx * dx);
    }
    g.leadingEdgeCos2 = n > 1 ? cos2 / (n - 1) : 1.0;
    const double mid0 = g.stripLeadingX.front() + 0.5 * g.stripChord.front();
    const double mid1 = g.stripLeadingX.back() + 0.5 * g.stripChord.back();
    g.midChordSweep = std::atan2(mid1 - mid0, g.stripY.back() - g.stripY.front());
    g.aspectRatio = g.area > 0.0 ? 2.0 * g.span * g.span / g.area : 0.0;
    return g;
}

MassProperties FinSet::computeMass() const {
    const FinGeometry g = geometry();
    const double r = bodyRadius();
    const double finMass = material_.density * thickness_ * g.area;
    const double tabMass = material_.density * thickness_ * tabHeight_ * tabLength_;
    MassProperties m;
    m.mass = count_ * (finMass + tabMass);
    if (m.mass <= 0.0) return m;
    const double cgX = (finMass * g.centroidX + tabMass * (tabPosition_ + 0.5 * tabLength_)) / (finMass + tabMass);
    m.cg = cgX;
    const double rho = r + g.centroidY;
    m.ixx = m.mass * (rho * rho + g.span * g.span / 18.0);
    m.iyy = m.mass * (0.5 * rho * rho + g.rootChord * g.rootChord / 18.0);
    return m;
}

void FinSet::writeProperties(json& j) const {
    j["count"] = count_;
    json pts = json::array();
    for (auto& p : outline_) pts.push_back({p.x, p.y});
    j["outline"] = pts;
    j["thickness"] = thickness_;
    j["crossSection"] = toString(crossSection_);
    if (cant_ != 0.0) j["cant"] = mathrix::rad2deg(cant_);
    if (rotation_ != 0.0) j["rotation"] = mathrix::rad2deg(rotation_);
    j["material"] = material_.toJson();
    j["finish"] = toString(finish_);
    if (tabHeight_ > 0.0) j["tab"] = {{"height", tabHeight_}, {"length", tabLength_}, {"position", tabPosition_}};
}

void FinSet::readProperties(const json& j) {
    thickness_ = j.value("thickness", thickness_);
    if (j.contains("outline")) {
        std::vector<Point> pts;
        for (const auto& p : j.at("outline")) pts.push_back({p.at(0).get<double>(), p.at(1).get<double>()});
        setOutline(pts);
    } else if (j.contains("trapezoidal")) {
        const auto& t = j.at("trapezoidal");
        const FinSet f = trapezoidal("", 1, t.at("rootChord").get<double>(), t.at("tipChord").get<double>(),
                                     t.at("span").get<double>(), t.value("sweep", 0.0), thickness_);
        setOutline(f.outline());
    } else if (j.contains("elliptical")) {
        const auto& t = j.at("elliptical");
        const FinSet f = elliptical("", 1, t.at("rootChord").get<double>(), t.at("span").get<double>(), thickness_);
        setOutline(f.outline());
    }
    setCount(j.value("count", count_));
    if (j.contains("crossSection")) crossSection_ = finCrossSectionFromString(j.at("crossSection").get<std::string>());
    cant_ = mathrix::deg2rad(j.value("cant", 0.0));
    rotation_ = mathrix::deg2rad(j.value("rotation", 0.0));
    if (j.contains("material")) material_ = Material::fromJson(j.at("material"));
    if (j.contains("finish")) finish_ = surfaceFinishFromString(j.at("finish").get<std::string>());
    if (j.contains("tab")) {
        const auto& t = j.at("tab");
        setTab(t.value("height", 0.0), t.value("length", 0.0), t.value("position", 0.0));
    }
}

// ------------------------------------------------------------------ LaunchLug

LaunchLug::LaunchLug(std::string name, double length, double outerRadius, double thickness)
    : Component(std::move(name)), length_(length), outerRadius_(outerRadius), thickness_(thickness) {}

double LaunchLug::frontalArea() const {
    const double ri = std::max(0.0, outerRadius_ - thickness_);
    return kPi * (outerRadius_ * outerRadius_ - ri * ri);
}

double LaunchLug::wettedArea() const {
    const double ri = std::max(0.0, outerRadius_ - thickness_);
    return 2.0 * kPi * (outerRadius_ + ri) * length_;
}

MassProperties LaunchLug::computeMass() const {
    MassProperties m;
    const double ri = std::max(0.0, outerRadius_ - thickness_);
    m.mass = material_.density * kPi * (outerRadius_ * outerRadius_ - ri * ri) * length_;
    m.cg = 0.5 * length_;
    // Sits on the body surface: model as a point mass for roll, slender rod for pitch.
    m.iyy = m.mass * length_ * length_ / 12.0;
    return m;
}

void LaunchLug::writeProperties(json& j) const {
    j["length"] = length_;
    j["outerRadius"] = outerRadius_;
    j["thickness"] = thickness_;
    j["material"] = material_.toJson();
}

void LaunchLug::readProperties(const json& j) {
    length_ = j.value("length", length_);
    outerRadius_ = detail::readRadius(j, "outerRadius", "outerDiameter", outerRadius_);
    thickness_ = j.value("thickness", thickness_);
    if (j.contains("material")) material_ = Material::fromJson(j.at("material"));
}

// ------------------------------------------------------------------ RailButton

RailButton::RailButton(std::string name, int count, double diameter, double height, double massEach)
    : Component(std::move(name)), count_(count), diameter_(diameter), height_(height), massEach_(massEach) {}

MassProperties RailButton::computeMass() const {
    MassProperties m;
    m.mass = count_ * massEach_;
    m.cg = 0.5 * diameter_;
    return m;
}

void RailButton::writeProperties(json& j) const {
    j["count"] = count_;
    j["diameter"] = diameter_;
    j["height"] = height_;
    j["massEach"] = massEach_;
}

void RailButton::readProperties(const json& j) {
    count_ = j.value("count", count_);
    diameter_ = j.value("diameter", diameter_);
    height_ = j.value("height", height_);
    massEach_ = j.value("massEach", massEach_);
}

}  // namespace rocketup
