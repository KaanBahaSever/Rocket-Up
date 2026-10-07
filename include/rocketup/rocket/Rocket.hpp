#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rocketup/components/BodyComponents.hpp"
#include "rocketup/components/FinSet.hpp"
#include "rocketup/components/InternalComponents.hpp"
#include "rocketup/components/Recovery.hpp"
#include "rocketup/sim/Trigger.hpp"

namespace rocketup {

/// How the drag coefficient is obtained.
enum class DragSource {
    Computed,  ///< component build-up (Barrowman / Niskanen), Mach & Reynolds dependent
    Table,     ///< user table Cd(Mach) or Cd(Mach, altitude), e.g. RASAero / CFD / wind tunnel
    Constant,  ///< single value
};
const char* toString(DragSource s);
DragSource dragSourceFromString(const std::string& s);

/// Rocket-level aerodynamic options.
struct AeroSettings {
    DragSource dragSource = DragSource::Computed;
    double constantCd = 0.5;
    /// Table: Cd[altitudeRow][machColumn]. One row = Mach-only table.
    std::vector<double> tableMach;
    std::vector<double> tableAltitude;  ///< m ASL, ascending (optional)
    std::vector<std::vector<double>> tableCd;
    mathrix::Interp tableInterpolation = mathrix::Interp::Linear;
    std::string tableSource;  ///< informative (file name)
    double cdMultiplier = 1.0;       ///< calibration / dispersion factor applied to the final Cd
    double referenceDiameter = 0.0;  ///< m, 0 = largest body diameter
    /// Exhaust plume fills the base while a motor burns (removes motor-area base drag).
    bool plumeReducesBaseDrag = true;

    /// Load a Cd table from CSV. Two layouts are accepted:
    ///  - columns "mach, cd"
    ///  - first column Mach, header row with altitudes, one Cd column per altitude
    void loadTable(const std::string& csvPath, mathrix::Interp interpolation = mathrix::Interp::Linear);
    void setTable(std::vector<double> mach, std::vector<double> cd);

    json toJson() const;
    static AeroSettings fromJson(const json& j, const std::string& baseDirectory = "");
};

/// Splits the rocket in two bodies when its trigger fires: everything from component
/// `aftSectionStart` (a stack component) to the tail becomes the aft body, the rest the
/// forward body. Both continue flying (and falling) independently.
struct Separation {
    std::string name = "Separation";
    std::string aftSectionStart;  ///< name of the first stack component of the aft section
    Trigger trigger = Trigger::apogee();
    double relativeSpeed = 1.0;   ///< m/s pushed apart along the axis
    std::string forwardName;      ///< name of the forward body (default: "<name> forward")
    std::string aftName;          ///< name of the aft body (default: keeps the parent's name)

    json toJson() const;
    static Separation fromJson(const json& j);
};

/// A component placed in the rocket: absolute position of its front end measured from the
/// nose tip (m), plus the stack component it belongs to.
struct PlacedComponent {
    const Component* component = nullptr;
    double position = 0.0;
    int stackIndex = 0;
};

/// The rocket design: an ordered stack of body components (nose to tail) with children,
/// separation events and aerodynamic settings. Pure data, safe to share between simulations.
class Rocket {
public:
    explicit Rocket(std::string name = "Rocket");
    Rocket(const Rocket& other);
    Rocket& operator=(const Rocket& other);
    Rocket(Rocket&&) noexcept = default;
    Rocket& operator=(Rocket&&) noexcept = default;

    const std::string& name() const { return name_; }
    void setName(std::string n) { name_ = std::move(n); }
    const std::string& designer() const { return designer_; }
    void setDesigner(std::string d) { designer_ = std::move(d); }
    const std::string& description() const { return description_; }
    void setDescription(std::string d) { description_ = std::move(d); }

    // ---- body stack
    /// Append a body component (nose cone, body tube, transition) at the tail.
    template <class T, class... Args>
    T& add(Args&&... args) {
        auto c = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *c;
        addStack(std::move(c));
        return ref;
    }
    SymmetricComponent& addStack(std::unique_ptr<Component> c);
    const std::vector<std::unique_ptr<SymmetricComponent>>& stack() const { return stack_; }
    std::unique_ptr<SymmetricComponent> removeStack(size_t index);

    Component* find(const std::string& name);
    const Component* find(const std::string& name) const;
    template <class T>
    T* findAs(const std::string& name) {
        return dynamic_cast<T*>(find(name));
    }

    /// Every component with its absolute front position, depth-first.
    std::vector<PlacedComponent> flatten() const;
    /// Absolute front position of the stack component `index`.
    double stackPosition(size_t index) const;
    int stackIndexOf(const std::string& name) const;

    double length() const;
    double maxRadius() const;
    double referenceDiameter() const;
    double referenceArea() const;

    std::vector<const MotorMount*> motorMounts() const;
    std::vector<const RecoveryDevice*> recoveryDevices() const;
    std::vector<const Payload*> payloads() const;

    // ---- separations
    std::vector<Separation>& separations() { return separations_; }
    const std::vector<Separation>& separations() const { return separations_; }
    Separation& addSeparation(Separation s);

    // ---- aerodynamics
    AeroSettings& aero() { return aero_; }
    const AeroSettings& aero() const { return aero_; }

    // ---- mass
    /// Mass properties (cg from the nose tip) with motors at `motorTime` s after ignition
    /// (negative: unburned) - convenient for design summaries.
    MassProperties massProperties(double motorTime = -1.0) const;
    MassProperties dryMassProperties() const;  ///< without propellant
    /// Override the mass/CG of the whole structure without motors (measured on a scale).
    void overrideDryMass(double mass, double cgFromNose);
    void clearDryMassOverride() { dryOverride_.reset(); }
    std::optional<MassProperties> dryMassOverride() const { return dryOverride_; }
    /// Structure-only mass properties (no motor), honouring the override.
    MassProperties structureMass() const;

    /// Consistency checks (empty stack, missing motor, ...). Returns human readable problems.
    std::vector<std::string> validate() const;

    json toJson() const;
    static Rocket fromJson(const json& j, const std::string& baseDirectory = "");
    /// Design files: `.rocketup` (JSON).
    void save(const std::string& path) const;
    static Rocket load(const std::string& path);

private:
    std::string name_, designer_, description_;
    std::vector<std::unique_ptr<SymmetricComponent>> stack_;
    std::vector<Separation> separations_;
    AeroSettings aero_;
    std::optional<MassProperties> dryOverride_;
};

}  // namespace rocketup
