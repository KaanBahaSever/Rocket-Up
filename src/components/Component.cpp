#include "rocketup/components/Component.hpp"

#include <map>
#include <mutex>
#include <nlohmann/json.hpp>

#include "rocketup/components/BodyComponents.hpp"
#include "rocketup/components/FinSet.hpp"
#include "rocketup/components/InternalComponents.hpp"
#include "rocketup/components/Recovery.hpp"

namespace rocketup {

// ------------------------------------------------------------------ MassProperties

MassProperties& MassProperties::operator+=(const MassProperties& o) {
    const double m = mass + o.mass;
    if (m <= 0.0) return *this;
    const double c = (mass * cg + o.mass * o.cg) / m;
    iyy = iyy + mass * (cg - c) * (cg - c) + o.iyy + o.mass * (o.cg - c) * (o.cg - c);
    ixx += o.ixx;
    mass = m;
    cg = c;
    return *this;
}

MassProperties MassProperties::scaledTo(double newMass) const {
    MassProperties r = *this;
    if (mass > 0.0) {
        const double f = newMass / mass;
        r.ixx *= f;
        r.iyy *= f;
    } else {
        r.ixx = r.iyy = 0.0;
    }
    r.mass = newMass;
    return r;
}

const char* toString(Anchor a) {
    switch (a) {
        case Anchor::Top: return "top";
        case Anchor::Middle: return "middle";
        case Anchor::Bottom: return "bottom";
    }
    return "top";
}

Anchor anchorFromString(const std::string& s) {
    if (s == "top" || s == "front") return Anchor::Top;
    if (s == "middle" || s == "center" || s == "centre") return Anchor::Middle;
    if (s == "bottom" || s == "aft" || s == "rear") return Anchor::Bottom;
    throw RocketUpError("unknown anchor '" + s + "' (use top, middle or bottom)");
}

// ------------------------------------------------------------------ Component

Component::Component(std::string name) : name_(std::move(name)) {}

Component::Component(const Component& o)
    : name_(o.name_),
      comment_(o.comment_),
      anchor_(o.anchor_),
      offset_(o.offset_),
      massOverride_(o.massOverride_),
      cgOverride_(o.cgOverride_),
      parent_(nullptr) {
    for (const auto& c : o.children_) {
        auto copy = c->clone();
        copy->parent_ = this;
        children_.push_back(std::move(copy));
    }
}

double Component::positionInParent() const {
    if (!parent_) return offset_;
    switch (anchor_) {
        case Anchor::Top: return offset_;
        case Anchor::Middle: return 0.5 * (parent_->length() - length()) + offset_;
        case Anchor::Bottom: return parent_->length() - length() + offset_;
    }
    return offset_;
}

MassProperties Component::mass() const {
    MassProperties m = computeMass();
    if (massOverride_) {
        if (m.mass <= 0.0) m.cg = 0.5 * length();
        m = m.scaledTo(*massOverride_);
    }
    if (cgOverride_) m.cg = *cgOverride_;
    return m;
}

MassProperties Component::totalMass() const {
    MassProperties m = mass();
    for (const auto& c : children_) m += c->totalMass().shifted(c->positionInParent());
    return m;
}

bool Component::acceptsChild(const Component&) const { return false; }

Component& Component::addChild(std::unique_ptr<Component> child) {
    if (!child) throw RocketUpError("cannot add a null component");
    if (child->isBody()) throw RocketUpError("'" + child->name() + "' is a body component: add it to the rocket stack");
    if (!acceptsChild(*child))
        throw RocketUpError("component '" + name_ + "' (" + type() + ") cannot hold '" + child->name() + "' (" +
                            child->type() + ")");
    child->parent_ = this;
    children_.push_back(std::move(child));
    return *children_.back();
}

std::unique_ptr<Component> Component::removeChild(const Component* child) {
    for (auto it = children_.begin(); it != children_.end(); ++it) {
        if (it->get() == child) {
            auto out = std::move(*it);
            children_.erase(it);
            out->parent_ = nullptr;
            return out;
        }
    }
    return nullptr;
}

Component* Component::find(const std::string& name) {
    if (name_ == name) return this;
    for (auto& c : children_)
        if (Component* f = c->find(name)) return f;
    return nullptr;
}

const Component* Component::find(const std::string& name) const { return const_cast<Component*>(this)->find(name); }

void Component::visit(const std::function<void(const Component&, int)>& fn, int depth) const {
    fn(*this, depth);
    for (const auto& c : children_) c->visit(fn, depth + 1);
}

// ------------------------------------------------------------------ serialization

namespace {
std::mutex& factoryMutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, Component::Factory>& factories() {
    static std::map<std::string, Component::Factory> f = {
        {"nose-cone", [] { return std::make_unique<NoseCone>(); }},
        {"body-tube", [] { return std::make_unique<BodyTube>(); }},
        {"transition", [] { return std::make_unique<Transition>(); }},
        {"fin-set", [] { return std::make_unique<FinSet>(); }},
        {"launch-lug", [] { return std::make_unique<LaunchLug>(); }},
        {"rail-button", [] { return std::make_unique<RailButton>(); }},
        {"inner-tube", [] { return std::make_unique<InnerTube>(); }},
        {"ring", [] { return std::make_unique<Ring>(); }},
        {"mass", [] { return std::make_unique<MassObject>(); }},
        {"motor-mount", [] { return std::make_unique<MotorMount>(); }},
        {"parachute", [] { return std::make_unique<Parachute>(); }},
        {"streamer", [] { return std::make_unique<Streamer>(); }},
        {"payload", [] { return std::make_unique<Payload>(); }},
    };
    return f;
}
}  // namespace

void Component::registerType(const std::string& type, Factory factory) {
    std::lock_guard<std::mutex> lock(factoryMutex());
    factories()[type] = std::move(factory);
}

json Component::toJson() const {
    json j = {{"type", type()}, {"name", name_}};
    if (!comment_.empty()) j["comment"] = comment_;
    if (!isBody()) j["position"] = {{"anchor", toString(anchor_)}, {"offset", offset_}};
    writeProperties(j);
    if (massOverride_) j["massOverride"] = *massOverride_;
    if (cgOverride_) j["cgOverride"] = *cgOverride_;
    if (!children_.empty()) {
        json c = json::array();
        for (const auto& ch : children_) c.push_back(ch->toJson());
        j["children"] = c;
    }
    return j;
}

std::unique_ptr<Component> Component::fromJson(const json& j) {
    const std::string type = j.at("type").get<std::string>();
    Factory f;
    {
        std::lock_guard<std::mutex> lock(factoryMutex());
        auto it = factories().find(type);
        if (it == factories().end()) throw RocketUpError("unknown component type '" + type + "'");
        f = it->second;
    }
    auto c = f();
    c->name_ = j.value("name", std::string(type));
    c->comment_ = j.value("comment", std::string());
    if (j.contains("position")) {
        const auto& p = j.at("position");
        if (p.is_number()) {
            c->anchor_ = Anchor::Top;
            c->offset_ = p.get<double>();
        } else {
            c->anchor_ = anchorFromString(p.value("anchor", std::string("top")));
            c->offset_ = p.value("offset", 0.0);
        }
    }
    try {
        c->readProperties(j);
    } catch (const json::exception& e) {
        throw RocketUpError("component '" + c->name_ + "': " + e.what());
    }
    if (j.contains("massOverride")) c->massOverride_ = j.at("massOverride").get<double>();
    if (j.contains("cgOverride")) c->cgOverride_ = j.at("cgOverride").get<double>();
    if (j.contains("children"))
        for (const auto& cj : j.at("children")) c->addChild(fromJson(cj));
    return c;
}

}  // namespace rocketup
