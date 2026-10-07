#pragma once

#include <string>
#include <vector>

#include "rocketup/rocket/Rocket.hpp"

namespace rocketup {

/// Free-stream conditions seen by the rocket.
struct FlightConditions {
    double mach = 0.0;
    double airspeed = 0.0;            ///< m/s
    double density = 1.225;           ///< kg/m^3
    double kinematicViscosity = 1.5e-5;  ///< m^2/s
    double altitudeAsl = 0.0;         ///< m (for altitude dependent Cd tables)
    double angleOfAttack = 0.0;       ///< rad, total angle between axis and relative wind, [0, pi]
    double rollRate = 0.0;            ///< rad/s
    double plumeArea = 0.0;           ///< m^2 of base covered by active motor exhaust

    double dynamicPressure() const { return 0.5 * density * airspeed * airspeed; }
};

/// Aerodynamic coefficients (reference area/length = rocket reference diameter).
struct AeroCoefficients {
    double cd = 0.0;          ///< zero-AoA drag coefficient actually used (after table/multiplier)
    double cdFriction = 0.0;  ///< computed breakdown (independent of the drag source)
    double cdPressure = 0.0;
    double cdBase = 0.0;
    double axial = 0.0;       ///< axial force coefficient at the current AoA
    double normal = 0.0;      ///< normal force coefficient at the current AoA
    double cnAlpha = 0.0;     ///< normal force slope (1/rad) at small AoA
    double cp = 0.0;          ///< centre of pressure, m from the nose tip
    double pitchDamping = 0.0;  ///< moment coefficient multiplier of (omega/v)^2
    double rollForcing = 0.0;   ///< roll moment coefficient from fin cant
    double rollDamping = 0.0;   ///< roll damping moment coefficient at the given roll rate
    double reynolds = 0.0;
    double referenceArea = 0.0;
    double referenceLength = 0.0;
    /// Drag along the relative wind at the AoA: C_A cos(a) + C_N sin(a).
    double totalDrag(double aoa) const;
};

/// Per-component contribution (for reports and the future GUI).
struct ComponentAero {
    std::string name;
    std::string type;
    double cnAlpha = 0.0;
    double cp = 0.0;  ///< m from nose tip
    double cdFriction = 0.0;
    double cdPressure = 0.0;
    double cdBase = 0.0;
};

/// Semi-empirical aerodynamics without CFD.
///
/// Normal force and centre of pressure: Barrowman's method extended with Galejs' body
/// lift term and compressibility corrections for fins (subsonic lifting-line, supersonic
/// linear theory, interpolated through the transonic range).
///
/// Drag: component build-up after Niskanen (2009, "Development of an Open Source model
/// rocket simulation software") as used by OpenRocket:
///   * skin friction from the Reynolds number (laminar/turbulent/roughness limited) with
///     compressibility corrections, body fineness and fin thickness form factors;
///   * pressure drag of nose cones/shoulders (shape dependent, wave drag above M 0.8),
///     boattails, fin leading/trailing edges, launch lugs and rail buttons;
///   * base drag (subsonic 0.12 + 0.13 M^2, supersonic 0.25/M), partly filled by the
///     motor plume while thrusting;
///   * angle-of-attack correction of the axial force.
/// The result is Mach, altitude (Reynolds) and AoA dependent. A user table or a constant
/// can replace the total drag while keeping the stability model.
class AeroModel {
public:
    /// Build the model for a set of placed components (a whole rocket or one section).
    AeroModel(const std::vector<PlacedComponent>& parts, const AeroSettings& settings, double referenceDiameter);
    explicit AeroModel(const Rocket& rocket);

    /// `cg` is measured from the nose tip of the original rocket (same frame as positions).
    AeroCoefficients compute(const FlightConditions& c, double cg) const;
    std::vector<ComponentAero> breakdown(const FlightConditions& c) const;

    bool hasBody() const { return !bodies_.empty(); }
    bool hasNose() const { return hasNose_; }
    bool hasFins() const { return !fins_.empty(); }
    double length() const { return length_; }
    double frontPosition() const { return front_; }  ///< nose tip of this section (m)
    double referenceArea() const { return refArea_; }
    double referenceDiameter() const { return refDiameter_; }
    /// Effective Cd*A (m^2) of the section tumbling end over end (used during descent).
    double tumblingDragArea() const { return tumbleCdA_; }
    /// Static margin in calibers for a given CG (positive = stable).
    double stabilityMargin(double cg, double mach = 0.3) const;

    // Standard correlations, exposed for tests.
    static double stagnationCd(double mach);
    static double baseCd(double mach);
    static double skinFrictionCoefficient(double reynolds, double mach, double roughness, double length);

private:
    struct Body {
        std::string name, type;
        double x0, length, rFore, rAft;
        double wettedArea, planformArea, planformCentroid, volume;
        double roughness;
        NoseShape shape = NoseShape::Conical;
        double shapeParameter = 0.0;
        bool isNose = false;
    };
    struct Fins {
        std::string name;
        double x0;  ///< root leading edge
        FinGeometry g;
        int count;
        double thickness, cant, bodyRadius, roughness;
        FinCrossSection cross;
    };
    struct Protrusion {
        std::string name, type;
        double x, frontalArea, wettedArea, roughness;
    };

    std::vector<Body> bodies_;
    std::vector<Fins> fins_;
    std::vector<Protrusion> protrusions_;
    AeroSettings settings_;
    mathrix::Table2D cdTable_;
    double refDiameter_ = 0.1, refArea_ = 0.00785;
    double length_ = 0.0, front_ = 0.0, maxRadius_ = 0.0, avgDiameter_ = 0.0;
    double tumbleCdA_ = 0.0;
    bool hasNose_ = false;

    void build(const std::vector<PlacedComponent>& parts);
    double finCnAlphaSingle(const Fins& f, double mach) const;
    double finCpFraction(const Fins& f, double mach) const;
    static double finSetFactor(int n);
    double nosePressureCd(const Body& b, double mach) const;
    void drag(const FlightConditions& c, double& friction, double& pressure, double& base,
              std::vector<ComponentAero>* detail) const;
};

}  // namespace rocketup
