#pragma once

#include <vector>

#include "rocketup/components/Component.hpp"
#include "rocketup/components/Material.hpp"

namespace rocketup {

/// Leading/trailing edge profile of the fin cross-section.
enum class FinCrossSection { Square, Rounded, Airfoil };
const char* toString(FinCrossSection c);
FinCrossSection finCrossSectionFromString(const std::string& s);

/// Planform quantities derived from the fin outline (all in the fin's local frame:
/// x along the root chord from the root leading edge, y spanwise from the root).
struct FinGeometry {
    double rootChord = 0.0;
    double tipChord = 0.0;
    double span = 0.0;
    double area = 0.0;          ///< one fin, one side
    double macLength = 0.0;     ///< mean aerodynamic chord
    double macLeadingX = 0.0;   ///< x of the MAC leading edge
    double macSpanY = 0.0;      ///< spanwise station of the MAC
    double midChordSweep = 0.0; ///< rad
    double leadingEdgeCos2 = 1.0;  ///< span-averaged cos^2 of the leading-edge sweep
    double centroidX = 0.0, centroidY = 0.0;
    double aspectRatio = 0.0;   ///< 2 s^2 / A
    std::vector<double> stripY, stripChord, stripLeadingX;  ///< spanwise strips (for roll damping)
};

/// A set of identical fins spaced evenly around the body. The outline is a polygon whose
/// first and last points lie on the root (y = 0), so trapezoidal, elliptical, swept, clipped
/// delta and fully free-form fins all share one model.
class FinSet final : public Component {
public:
    struct Point {
        double x, y;
    };

    FinSet(std::string name = "Fins", int count = 4, std::vector<Point> outline = {}, double thickness = 0.003);
    ROCKETUP_COMPONENT_CLONE(FinSet)
    const char* type() const override { return "fin-set"; }
    bool isExternal() const override { return true; }
    double length() const override;
    double outerRadius() const override;

    static FinSet trapezoidal(std::string name, int count, double rootChord, double tipChord, double span,
                              double sweepLength, double thickness);
    static FinSet elliptical(std::string name, int count, double rootChord, double span, double thickness);
    static FinSet freeform(std::string name, int count, std::vector<Point> outline, double thickness);

    int count() const { return count_; }
    FinSet& setCount(int n);
    const std::vector<Point>& outline() const { return outline_; }
    FinSet& setOutline(std::vector<Point> pts);
    double thickness() const { return thickness_; }
    FinSet& setThickness(double t) { thickness_ = t; return *this; }
    FinCrossSection crossSection() const { return crossSection_; }
    FinSet& setCrossSection(FinCrossSection c) { crossSection_ = c; return *this; }
    double cantAngle() const { return cant_; }  ///< rad
    FinSet& setCantAngle(double rad) { cant_ = rad; return *this; }
    double rotation() const { return rotation_; }  ///< rad, angular position of the first fin
    FinSet& setRotation(double rad) { rotation_ = rad; return *this; }
    const Material& material() const { return material_; }
    FinSet& setMaterial(Material m) { material_ = std::move(m); return *this; }
    SurfaceFinish finish() const { return finish_; }
    FinSet& setFinish(SurfaceFinish f) { finish_ = f; return *this; }
    /// Fin tab glued into the body tube (mass only).
    FinSet& setTab(double height, double length, double fromRootLeadingEdge);

    /// Body radius at the fin root (taken from the parent body component).
    double bodyRadius() const;
    FinGeometry geometry() const;
    double planformArea() const;  ///< exact polygon area of one fin

protected:
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    int count_;
    std::vector<Point> outline_;
    double thickness_;
    FinCrossSection crossSection_ = FinCrossSection::Square;
    double cant_ = 0.0;
    double rotation_ = 0.0;
    Material material_ = materials::fiberglass();
    SurfaceFinish finish_ = SurfaceFinish::Regular;
    double tabHeight_ = 0.0, tabLength_ = 0.0, tabPosition_ = 0.0;
};

/// Launch lug (tube) sliding on a rod.
class LaunchLug final : public Component {
public:
    LaunchLug(std::string name = "Launch lug", double length = 0.05, double outerRadius = 0.005,
              double thickness = 0.001);
    ROCKETUP_COMPONENT_CLONE(LaunchLug)
    const char* type() const override { return "launch-lug"; }
    bool isExternal() const override { return true; }
    double length() const override { return length_; }
    double outerRadius() const override { return outerRadius_; }
    double thickness() const { return thickness_; }
    const Material& material() const { return material_; }
    LaunchLug& setMaterial(Material m) { material_ = std::move(m); return *this; }
    SurfaceFinish finish() const { return SurfaceFinish::Regular; }
    double frontalArea() const;
    double wettedArea() const;

protected:
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    double length_, outerRadius_, thickness_;
    Material material_ = materials::aluminum();
};

/// Rail buttons (guides for a slotted rail).
class RailButton final : public Component {
public:
    RailButton(std::string name = "Rail buttons", int count = 2, double diameter = 0.0115, double height = 0.008,
               double massEach = 0.005);
    ROCKETUP_COMPONENT_CLONE(RailButton)
    const char* type() const override { return "rail-button"; }
    bool isExternal() const override { return true; }
    double length() const override { return diameter_; }
    double outerRadius() const override { return 0.5 * diameter_; }
    int count() const { return count_; }
    double frontalArea() const { return count_ * diameter_ * height_; }
    double height() const { return height_; }

protected:
    MassProperties computeMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    int count_;
    double diameter_, height_, massEach_;
};

}  // namespace rocketup
