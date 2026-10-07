#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "rocketup/core/Types.hpp"

namespace rocketup {

/// Axisymmetric mass properties. `cg` is an axial coordinate whose origin depends on the
/// producer (component front for `Component::mass()`, nose tip for rocket-level values).
struct MassProperties {
    double mass = 0.0;  ///< kg
    double cg = 0.0;    ///< m
    double ixx = 0.0;   ///< roll (axial) moment of inertia, kg m^2
    double iyy = 0.0;   ///< pitch/yaw moment of inertia about the CG, kg m^2

    MassProperties shifted(double dx) const {
        MassProperties r = *this;
        r.cg += dx;
        return r;
    }
    /// Combine two mass distributions (parallel axis theorem).
    MassProperties& operator+=(const MassProperties& o);
    MassProperties operator+(const MassProperties& o) const {
        MassProperties r = *this;
        r += o;
        return r;
    }
    /// Scale mass and inertias (e.g. for mass overrides).
    MassProperties scaledTo(double newMass) const;
};

/// Where a child component sits inside its parent.
enum class Anchor {
    Top,     ///< offset measured from parent front to child front (positive aft)
    Middle,  ///< offset between the centres
    Bottom,  ///< offset measured from parent aft end to child aft end (positive aft)
};
const char* toString(Anchor a);
Anchor anchorFromString(const std::string& s);

/// Base class of every rocket part. Components form a tree: body components (nose cone,
/// tubes, transitions) are stacked by the Rocket and carry children (fins, internal parts,
/// parachutes, payloads, motor mounts). Components are pure design data: simulations never
/// modify them, so one design can be simulated many times in parallel.
class Component {
public:
    explicit Component(std::string name = "");
    virtual ~Component() = default;
    Component& operator=(const Component&) = delete;

    /// Stable type identifier used in design files ("nose-cone", "body-tube", ...).
    virtual const char* type() const = 0;
    /// Deep copy, including children.
    virtual std::unique_ptr<Component> clone() const = 0;

    /// Part of the outer mold line (affects aerodynamics).
    virtual bool isExternal() const { return false; }
    /// Axisymmetric stack component (nose cone, body tube, transition).
    virtual bool isBody() const { return false; }
    /// Axial length used for placement (m).
    virtual double length() const = 0;
    /// Largest outer radius (m), used for drawings and "auto" sizes.
    virtual double outerRadius() const { return 0.0; }

    const std::string& name() const { return name_; }
    void setName(std::string n) { name_ = std::move(n); }
    const std::string& comment() const { return comment_; }
    void setComment(std::string c) { comment_ = std::move(c); }

    // ---- placement (ignored for stacked body components)
    Anchor anchor() const { return anchor_; }
    double offset() const { return offset_; }
    Component& setPosition(Anchor anchor, double offset) {
        anchor_ = anchor;
        offset_ = offset;
        return *this;
    }
    /// Front of this component measured from the front of its parent (m).
    double positionInParent() const;

    // ---- mass
    /// Mass properties of this component alone (children excluded), local frame: cg from
    /// the component front. Overrides are applied.
    MassProperties mass() const;
    void overrideMass(double kg) { massOverride_ = kg; }
    void overrideCg(double fromFront) { cgOverride_ = fromFront; }
    void clearOverrides() {
        massOverride_.reset();
        cgOverride_.reset();
    }
    std::optional<double> massOverride() const { return massOverride_; }
    std::optional<double> cgOverride() const { return cgOverride_; }
    /// This component plus all children, local frame.
    MassProperties totalMass() const;

    // ---- tree
    Component* parent() const { return parent_; }
    const std::vector<std::unique_ptr<Component>>& children() const { return children_; }
    virtual bool acceptsChild(const Component& child) const;
    Component& addChild(std::unique_ptr<Component> child);
    template <class T, class... Args>
    T& add(Args&&... args) {
        static_assert(std::is_base_of_v<Component, T>, "T must derive from Component");
        return static_cast<T&>(addChild(std::make_unique<T>(std::forward<Args>(args)...)));
    }
    std::unique_ptr<Component> removeChild(const Component* child);
    /// Depth-first search by name (this component included).
    Component* find(const std::string& name);
    const Component* find(const std::string& name) const;
    void visit(const std::function<void(const Component&, int depth)>& fn, int depth = 0) const;

    // ---- serialization
    json toJson() const;
    static std::unique_ptr<Component> fromJson(const json& j);
    using Factory = std::function<std::unique_ptr<Component>()>;
    /// Register additional component types (plugins / GUI extensions).
    static void registerType(const std::string& type, Factory factory);

protected:
    Component(const Component& other);  ///< deep copy (used by clone())
    virtual MassProperties computeMass() const = 0;
    virtual void writeProperties(json& j) const = 0;
    virtual void readProperties(const json& j) = 0;

private:
    std::string name_;
    std::string comment_;
    Anchor anchor_ = Anchor::Top;
    double offset_ = 0.0;
    std::optional<double> massOverride_;
    std::optional<double> cgOverride_;
    Component* parent_ = nullptr;
    std::vector<std::unique_ptr<Component>> children_;
};

/// Helper: implements clone() for a concrete component type.
#define ROCKETUP_COMPONENT_CLONE(Type) \
    std::unique_ptr<Component> clone() const override { return std::unique_ptr<Component>(new Type(*this)); }

}  // namespace rocketup
