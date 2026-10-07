#include "rocketup/components/Material.hpp"

#include <nlohmann/json.hpp>

#include "rocketup/core/Io.hpp"

namespace rocketup {

namespace {
const char* kindName(Material::Kind k) {
    switch (k) {
        case Material::Kind::Bulk: return "bulk";
        case Material::Kind::Surface: return "surface";
        case Material::Kind::Line: return "line";
    }
    return "bulk";
}
}  // namespace

json Material::toJson() const { return {{"name", name}, {"density", density}, {"kind", kindName(kind)}}; }

Material Material::fromJson(const json& j) {
    if (j.is_string()) return materials::byName(j.get<std::string>());
    Material m;
    if (j.contains("name") && !j.contains("density")) return materials::byName(j.at("name").get<std::string>());
    m.name = j.value("name", std::string("Custom"));
    m.density = j.at("density").get<double>();
    const std::string k = j.value("kind", std::string("bulk"));
    m.kind = k == "surface" ? Kind::Surface : (k == "line" ? Kind::Line : Kind::Bulk);
    return m;
}

namespace materials {
Material fiberglass() { return {"Fiberglass", 1850.0, Material::Kind::Bulk}; }
Material carbonFiber() { return {"Carbon fiber", 1780.0, Material::Kind::Bulk}; }
Material aluminum() { return {"Aluminum", 2700.0, Material::Kind::Bulk}; }
Material cardboard() { return {"Cardboard", 680.0, Material::Kind::Bulk}; }
Material kraftPhenolic() { return {"Kraft phenolic", 950.0, Material::Kind::Bulk}; }
Material blueTube() { return {"Blue tube", 1300.0, Material::Kind::Bulk}; }
Material plywood() { return {"Plywood (birch)", 630.0, Material::Kind::Bulk}; }
Material balsa() { return {"Balsa", 170.0, Material::Kind::Bulk}; }
Material pla() { return {"PLA", 1250.0, Material::Kind::Bulk}; }
Material abs() { return {"ABS", 1040.0, Material::Kind::Bulk}; }
Material petg() { return {"PETG", 1270.0, Material::Kind::Bulk}; }
Material steel() { return {"Steel", 7850.0, Material::Kind::Bulk}; }
Material ripstopNylon() { return {"Ripstop nylon", 0.067, Material::Kind::Surface}; }
Material kevlarCord() { return {"Kevlar cord", 0.0034, Material::Kind::Line}; }
Material nylonCord() { return {"Nylon cord", 0.0018, Material::Kind::Line}; }

std::vector<Material> library() {
    return {fiberglass(), carbonFiber(), aluminum(), cardboard(), kraftPhenolic(), blueTube(), plywood(), balsa(),
            pla(),        abs(),         petg(),     steel(),     ripstopNylon(),  kevlarCord(), nylonCord()};
}

Material byName(const std::string& name) {
    const std::string n = io::toLower(name);
    for (auto& m : library())
        if (io::toLower(m.name) == n) return m;
    if (n == "aluminium") return aluminum();
    if (n == "carbon") return carbonFiber();
    if (n == "glass fiber" || n == "gfrp") return fiberglass();
    throw RocketUpError("unknown material '" + name + "'");
}
}  // namespace materials

double roughnessHeight(SurfaceFinish f) {
    switch (f) {
        case SurfaceFinish::Polished: return 2e-6;
        case SurfaceFinish::Smooth: return 20e-6;
        case SurfaceFinish::Regular: return 60e-6;
        case SurfaceFinish::Unfinished: return 150e-6;
        case SurfaceFinish::Rough: return 500e-6;
    }
    return 60e-6;
}

const char* toString(SurfaceFinish f) {
    switch (f) {
        case SurfaceFinish::Polished: return "polished";
        case SurfaceFinish::Smooth: return "smooth";
        case SurfaceFinish::Regular: return "regular";
        case SurfaceFinish::Unfinished: return "unfinished";
        case SurfaceFinish::Rough: return "rough";
    }
    return "regular";
}

SurfaceFinish surfaceFinishFromString(const std::string& s) {
    const std::string n = io::toLower(s);
    if (n == "polished" || n == "mirror") return SurfaceFinish::Polished;
    if (n == "smooth") return SurfaceFinish::Smooth;
    if (n == "regular" || n == "normal" || n == "paint") return SurfaceFinish::Regular;
    if (n == "unfinished") return SurfaceFinish::Unfinished;
    if (n == "rough") return SurfaceFinish::Rough;
    throw RocketUpError("unknown surface finish '" + s + "'");
}

}  // namespace rocketup
