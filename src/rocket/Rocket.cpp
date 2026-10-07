#include "rocketup/rocket/Rocket.hpp"

#include <cmath>
#include <functional>
#include <nlohmann/json.hpp>

#include "../components/JsonHelpers.hpp"
#include "rocketup/core/Io.hpp"

namespace rocketup {

const char* toString(DragSource s) {
    switch (s) {
        case DragSource::Computed: return "computed";
        case DragSource::Table: return "table";
        case DragSource::Constant: return "constant";
    }
    return "computed";
}

DragSource dragSourceFromString(const std::string& s) {
    if (s == "computed" || s == "barrowman" || s == "auto") return DragSource::Computed;
    if (s == "table") return DragSource::Table;
    if (s == "constant") return DragSource::Constant;
    throw RocketUpError("unknown drag source '" + s + "'");
}

// ------------------------------------------------------------------ AeroSettings

void AeroSettings::setTable(std::vector<double> mach, std::vector<double> cd) {
    if (mach.size() != cd.size() || mach.empty()) throw RocketUpError("Cd table: Mach and Cd sizes differ");
    tableMach = std::move(mach);
    tableAltitude.clear();
    tableCd = {std::move(cd)};
    dragSource = DragSource::Table;
}

void AeroSettings::loadTable(const std::string& csvPath, mathrix::Interp interpolation) {
    const auto t = io::readCsvFile(csvPath);
    if (t.rows.empty() || t.columnCount() < 2) throw RocketUpError("Cd table '" + csvPath + "' needs >= 2 columns");
    tableInterpolation = interpolation;
    tableSource = csvPath;
    tableMach = t.column(0);
    tableCd.clear();
    tableAltitude.clear();
    if (t.columnCount() == 2) {
        tableCd.push_back(t.column(1));
    } else {
        // Altitude header: numbers in header cells 1..n ("0", "3000 m", ...).
        for (size_t c = 1; c < t.columnCount(); ++c) {
            double alt = 0.0;
            std::string h = c < t.header.size() ? t.header[c] : std::string();
            std::string digits;
            for (char ch : h)
                if (std::isdigit(static_cast<unsigned char>(ch)) || ch == '.' || ch == '-') digits.push_back(ch);
            if (!io::tryParseDouble(digits, alt))
                throw RocketUpError("Cd table '" + csvPath + "': header must list the altitude of each Cd column");
            tableAltitude.push_back(alt);
            tableCd.push_back(t.column(c));
        }
    }
    dragSource = DragSource::Table;
}

json AeroSettings::toJson() const {
    json j = {{"dragSource", toString(dragSource)}, {"cdMultiplier", cdMultiplier}, {"plumeReducesBaseDrag", plumeReducesBaseDrag}};
    if (dragSource == DragSource::Constant) j["constantCd"] = constantCd;
    if (!tableMach.empty()) {
        j["table"] = {{"mach", tableMach}, {"cd", tableCd}, {"interpolation", mathrix::toString(tableInterpolation)}};
        if (!tableAltitude.empty()) j["table"]["altitude"] = tableAltitude;
        if (!tableSource.empty()) j["table"]["source"] = tableSource;
    }
    if (referenceDiameter > 0.0) j["referenceDiameter"] = referenceDiameter;
    return j;
}

AeroSettings AeroSettings::fromJson(const json& j, const std::string& baseDirectory) {
    AeroSettings a;
    if (j.contains("dragSource")) a.dragSource = dragSourceFromString(j.at("dragSource").get<std::string>());
    a.constantCd = j.value("constantCd", a.constantCd);
    a.cdMultiplier = j.value("cdMultiplier", 1.0);
    a.referenceDiameter = j.value("referenceDiameter", 0.0);
    a.plumeReducesBaseDrag = j.value("plumeReducesBaseDrag", true);
    if (j.contains("table")) {
        const auto& t = j.at("table");
        const auto interp = mathrix::interpFromString(t.value("interpolation", std::string("linear")));
        if (t.contains("file")) {
            const DragSource keep = a.dragSource;
            a.loadTable(io::resolvePath(t.at("file").get<std::string>(), baseDirectory), interp);
            a.dragSource = j.contains("dragSource") ? keep : DragSource::Table;
        } else {
            a.tableMach = t.at("mach").get<std::vector<double>>();
            const auto& cd = t.at("cd");
            if (!cd.empty() && cd.at(0).is_number()) a.tableCd = {cd.get<std::vector<double>>()};
            else a.tableCd = cd.get<std::vector<std::vector<double>>>();
            a.tableAltitude = t.value("altitude", std::vector<double>{});
            a.tableInterpolation = interp;
            a.tableSource = t.value("source", std::string());
        }
    }
    return a;
}

// ------------------------------------------------------------------ Separation

json Separation::toJson() const {
    json j = {{"name", name}, {"aftSectionStart", aftSectionStart}, {"trigger", trigger.toJson()},
              {"relativeSpeed", relativeSpeed}};
    if (!forwardName.empty()) j["forwardName"] = forwardName;
    if (!aftName.empty()) j["aftName"] = aftName;
    return j;
}

Separation Separation::fromJson(const json& j) {
    Separation s;
    s.name = j.value("name", s.name);
    s.aftSectionStart = j.at("aftSectionStart").get<std::string>();
    if (j.contains("trigger")) s.trigger = Trigger::fromJson(j.at("trigger"));
    s.relativeSpeed = j.value("relativeSpeed", s.relativeSpeed);
    s.forwardName = j.value("forwardName", std::string());
    s.aftName = j.value("aftName", std::string());
    return s;
}

// ------------------------------------------------------------------ Rocket

Rocket::Rocket(std::string name) : name_(std::move(name)) {}

Rocket::Rocket(const Rocket& o)
    : name_(o.name_),
      designer_(o.designer_),
      description_(o.description_),
      separations_(o.separations_),
      aero_(o.aero_),
      dryOverride_(o.dryOverride_) {
    for (const auto& c : o.stack_) {
        auto copy = c->clone();
        stack_.emplace_back(static_cast<SymmetricComponent*>(copy.release()));
    }
}

Rocket& Rocket::operator=(const Rocket& o) {
    if (this != &o) {
        Rocket tmp(o);
        *this = std::move(tmp);
    }
    return *this;
}

SymmetricComponent& Rocket::addStack(std::unique_ptr<Component> c) {
    auto* s = dynamic_cast<SymmetricComponent*>(c.get());
    if (!s) throw RocketUpError("only body components (nose cone, body tube, transition) can be stacked");
    c.release();
    stack_.emplace_back(s);
    return *s;
}

std::unique_ptr<SymmetricComponent> Rocket::removeStack(size_t index) {
    if (index >= stack_.size()) return nullptr;
    auto out = std::move(stack_[index]);
    stack_.erase(stack_.begin() + static_cast<std::ptrdiff_t>(index));
    return out;
}

Component* Rocket::find(const std::string& name) {
    for (auto& s : stack_)
        if (Component* c = s->find(name)) return c;
    return nullptr;
}

const Component* Rocket::find(const std::string& name) const { return const_cast<Rocket*>(this)->find(name); }

double Rocket::stackPosition(size_t index) const {
    double x = 0.0;
    for (size_t i = 0; i < index && i < stack_.size(); ++i) x += stack_[i]->length();
    return x;
}

int Rocket::stackIndexOf(const std::string& name) const {
    for (size_t i = 0; i < stack_.size(); ++i)
        if (stack_[i]->name() == name) return static_cast<int>(i);
    return -1;
}

std::vector<PlacedComponent> Rocket::flatten() const {
    std::vector<PlacedComponent> out;
    std::function<void(const Component&, double, int)> walk = [&](const Component& c, double x, int idx) {
        out.push_back({&c, x, idx});
        for (const auto& ch : c.children()) walk(*ch, x + ch->positionInParent(), idx);
    };
    double x = 0.0;
    for (size_t i = 0; i < stack_.size(); ++i) {
        walk(*stack_[i], x, static_cast<int>(i));
        x += stack_[i]->length();
    }
    return out;
}

double Rocket::length() const { return stackPosition(stack_.size()); }

double Rocket::maxRadius() const {
    double r = 0.0;
    for (const auto& s : stack_) r = std::max(r, s->outerRadius());
    return r;
}

double Rocket::referenceDiameter() const { return aero_.referenceDiameter > 0.0 ? aero_.referenceDiameter : 2.0 * maxRadius(); }

double Rocket::referenceArea() const {
    const double d = referenceDiameter();
    return mathrix::kPi * d * d / 4.0;
}

std::vector<const MotorMount*> Rocket::motorMounts() const {
    std::vector<const MotorMount*> out;
    for (const auto& p : flatten())
        if (auto* m = dynamic_cast<const MotorMount*>(p.component)) out.push_back(m);
    return out;
}

std::vector<const RecoveryDevice*> Rocket::recoveryDevices() const {
    std::vector<const RecoveryDevice*> out;
    for (const auto& p : flatten())
        if (auto* m = dynamic_cast<const RecoveryDevice*>(p.component)) out.push_back(m);
    return out;
}

std::vector<const Payload*> Rocket::payloads() const {
    std::vector<const Payload*> out;
    for (const auto& p : flatten())
        if (auto* m = dynamic_cast<const Payload*>(p.component)) out.push_back(m);
    return out;
}

Separation& Rocket::addSeparation(Separation s) {
    separations_.push_back(std::move(s));
    return separations_.back();
}

MassProperties Rocket::structureMass() const {
    if (dryOverride_) return *dryOverride_;
    MassProperties m;
    for (const auto& p : flatten()) m += p.component->mass().shifted(p.position);
    return m;
}

MassProperties Rocket::massProperties(double motorTime) const {
    MassProperties m = structureMass();
    for (const auto& p : flatten())
        if (auto* mm = dynamic_cast<const MotorMount*>(p.component)) m += mm->motorMass(motorTime).shifted(p.position);
    return m;
}

MassProperties Rocket::dryMassProperties() const { return massProperties(1e9); }

void Rocket::overrideDryMass(double mass, double cgFromNose) {
    MassProperties computed;
    for (const auto& p : flatten()) computed += p.component->mass().shifted(p.position);
    MassProperties m = computed.scaledTo(mass);
    m.cg = cgFromNose;
    dryOverride_ = m;
}

std::vector<std::string> Rocket::validate() const {
    std::vector<std::string> issues;
    if (stack_.empty()) issues.push_back("rocket has no body components");
    else if (!dynamic_cast<const NoseCone*>(stack_.front().get()))
        issues.push_back("first stack component is not a nose cone");
    bool hasMotor = false;
    for (auto* m : motorMounts()) hasMotor |= m->hasMotor();
    if (!hasMotor) issues.push_back("no motor installed");
    if (recoveryDevices().empty()) issues.push_back("no recovery device (ballistic descent)");
    for (const auto& s : separations_)
        if (stackIndexOf(s.aftSectionStart) <= 0)
            issues.push_back("separation '" + s.name + "': '" + s.aftSectionStart +
                             "' is not a stack component after the first one");
    if (aero_.dragSource == DragSource::Table && (aero_.tableMach.empty() || aero_.tableCd.empty()))
        issues.push_back("drag source is 'table' but no Cd table is loaded");
    return issues;
}

json Rocket::toJson() const {
    json stack = json::array();
    for (const auto& s : stack_) stack.push_back(s->toJson());
    json seps = json::array();
    for (const auto& s : separations_) seps.push_back(s.toJson());
    json j = {{"format", "rocketup-rocket"}, {"version", 1}, {"name", name_}, {"stack", stack},
              {"separations", seps}, {"aerodynamics", aero_.toJson()}};
    if (!designer_.empty()) j["designer"] = designer_;
    if (!description_.empty()) j["description"] = description_;
    if (dryOverride_)
        j["dryMassOverride"] = {{"mass", dryOverride_->mass}, {"cg", dryOverride_->cg}, {"ixx", dryOverride_->ixx},
                                {"iyy", dryOverride_->iyy}};
    return j;
}

Rocket Rocket::fromJson(const json& j, const std::string& baseDirectory) {
    detail::BaseDirectoryScope scope(baseDirectory);
    Rocket r(j.value("name", std::string("Rocket")));
    r.designer_ = j.value("designer", std::string());
    r.description_ = j.value("description", std::string());
    for (const auto& c : j.at("stack")) r.addStack(Component::fromJson(c));
    if (j.contains("separations"))
        for (const auto& s : j.at("separations")) r.separations_.push_back(Separation::fromJson(s));
    if (j.contains("aerodynamics")) r.aero_ = AeroSettings::fromJson(j.at("aerodynamics"), baseDirectory);
    if (j.contains("dryMassOverride")) {
        const auto& o = j.at("dryMassOverride");
        if (o.contains("ixx")) {
            MassProperties m;
            m.mass = o.at("mass").get<double>();
            m.cg = o.at("cg").get<double>();
            m.ixx = o.at("ixx").get<double>();
            m.iyy = o.at("iyy").get<double>();
            r.dryOverride_ = m;
        } else {
            r.overrideDryMass(o.at("mass").get<double>(), o.at("cg").get<double>());
        }
    }
    return r;
}

void Rocket::save(const std::string& path) const { io::writeJsonFile(path, toJson()); }

Rocket Rocket::load(const std::string& path) { return fromJson(io::readJsonFile(path), io::directoryOf(path)); }

}  // namespace rocketup
