#pragma once

#include <algorithm>

#include "rocketup/components/Component.hpp"
#include "rocketup/components/Material.hpp"

namespace rocketup {

/// Nose cone / transition profile families.
enum class NoseShape {
    Conical,
    Ogive,        ///< tangent ogive
    Ellipsoid,
    PowerSeries,  ///< r = R (x/L)^n, parameter n in (0, 1]
    Parabolic,    ///< parabolic series, parameter K in [0, 1]
    Haack,        ///< Haack series, parameter C (0 = Von Karman, 1/3 = LV-Haack)
};
const char* toString(NoseShape s);
NoseShape noseShapeFromString(const std::string& s);
/// Normalised profile r/R at normalised station xi = x/L (0 = tip, 1 = base).
double noseProfile(NoseShape shape, double parameter, double xi);

/// Axisymmetric shell (or solid) of revolution: base class of the body stack.
class SymmetricComponent : public Component {
public:
    bool isExternal() const override { return true; }
    bool isBody() const override { return true; }
    double length() const override { return length_; }
    double outerRadius() const override;
    bool acceptsChild(const Component& child) const override;

    /// Outer radius at local axial station x in [0, length].
    virtual double radiusAt(double x) const = 0;
    double foreRadius() const { return radiusAt(0.0); }
    double aftRadius() const { return radiusAt(length_); }

    SymmetricComponent& setLength(double l);
    double thickness() const { return thickness_; }
    SymmetricComponent& setThickness(double t) { thickness_ = t; return *this; }
    bool filled() const { return filled_; }
    SymmetricComponent& setFilled(bool f) { filled_ = f; return *this; }
    const Material& material() const { return material_; }
    SymmetricComponent& setMaterial(Material m) { material_ = std::move(m); return *this; }
    SurfaceFinish finish() const { return finish_; }
    SymmetricComponent& setFinish(SurfaceFinish f) { finish_ = f; return *this; }

    // ---- geometry integrals (numerical, 200 stations)
    double wettedArea() const;
    double planformArea() const;      ///< side-view projected area
    double planformCentroid() const;  ///< local x of the planform centroid
    double volume() const;            ///< enclosed volume

protected:
    SymmetricComponent(std::string name, double length);
    SymmetricComponent(const SymmetricComponent&) = default;
    MassProperties computeMass() const override;
    virtual MassProperties extraMass() const { return {}; }
    void writeCommon(json& j) const;
    void readCommon(const json& j);

    double length_;
    double thickness_ = 0.002;
    bool filled_ = false;
    Material material_ = materials::fiberglass();
    SurfaceFinish finish_ = SurfaceFinish::Regular;
};

class NoseCone final : public SymmetricComponent {
public:
    NoseCone(std::string name = "Nose cone", NoseShape shape = NoseShape::Ogive, double length = 0.3,
             double baseRadius = 0.05, double shapeParameter = 0.5);
    ROCKETUP_COMPONENT_CLONE(NoseCone)
    const char* type() const override { return "nose-cone"; }
    double radiusAt(double x) const override;

    NoseShape shape() const { return shape_; }
    NoseCone& setShape(NoseShape s, double parameter) { shape_ = s; parameter_ = parameter; return *this; }
    double shapeParameter() const { return parameter_; }
    double baseRadius() const { return baseRadius_; }
    NoseCone& setBaseRadius(double r) { baseRadius_ = r; return *this; }
    /// Shoulder that slides into the next tube (adds mass, no drag).
    NoseCone& setShoulder(double length, double radius, double thickness);
    double shoulderLength() const { return shoulderLength_; }

protected:
    MassProperties extraMass() const override;
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    NoseShape shape_;
    double baseRadius_;
    double parameter_;
    double shoulderLength_ = 0.0, shoulderRadius_ = 0.0, shoulderThickness_ = 0.0;
};

class BodyTube final : public SymmetricComponent {
public:
    BodyTube(std::string name = "Body tube", double length = 0.5, double radius = 0.05, double thickness = 0.002);
    ROCKETUP_COMPONENT_CLONE(BodyTube)
    const char* type() const override { return "body-tube"; }
    double radiusAt(double) const override { return radius_; }
    double radius() const { return radius_; }
    BodyTube& setRadius(double r) { radius_ = r; return *this; }
    double innerRadius() const { return filled_ ? 0.0 : std::max(0.0, radius_ - thickness_); }

protected:
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    double radius_;
};

/// Shoulder (diameter increase) or boattail (decrease) between two tubes.
class Transition final : public SymmetricComponent {
public:
    Transition(std::string name = "Transition", double length = 0.1, double foreRadius = 0.05,
               double aftRadius = 0.04, NoseShape shape = NoseShape::Conical, double shapeParameter = 0.0);
    ROCKETUP_COMPONENT_CLONE(Transition)
    const char* type() const override { return "transition"; }
    double radiusAt(double x) const override;
    Transition& setRadii(double fore, double aft) { fore_ = fore; aft_ = aft; return *this; }
    NoseShape shape() const { return shape_; }
    double shapeParameter() const { return parameter_; }

protected:
    void writeProperties(json& j) const override;
    void readProperties(const json& j) override;

private:
    double fore_, aft_;
    NoseShape shape_;
    double parameter_;
};

}  // namespace rocketup
