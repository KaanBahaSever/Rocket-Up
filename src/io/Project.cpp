#include "rocketup/io/Project.hpp"

#include <nlohmann/json.hpp>

#include "rocketup/core/Io.hpp"

namespace rocketup {

json Project::toJson() const {
    return {{"format", "rocketup-project"},
            {"version", 1},
            {"name", name},
            {"rocket", rocket.toJson()},
            {"environment", environment.toJson()},
            {"simulation", options.toJson()},
            {"output", {{"directory", outputDirectory}, {"csvInterval", csvInterval}}}};
}

Project Project::fromJson(const json& j, const std::string& base) {
    Project p;
    p.name = j.value("name", std::string());
    const auto& r = j.at("rocket");
    if (r.is_string()) p.rocket = Rocket::load(io::resolvePath(r.get<std::string>(), base));
    else p.rocket = Rocket::fromJson(r, base);
    if (j.contains("environment")) {
        const auto& e = j.at("environment");
        if (e.is_string()) p.environment = Environment::loadFile(io::resolvePath(e.get<std::string>(), base));
        else p.environment = Environment::fromJson(e, base);
    }
    if (j.contains("simulation")) p.options = SimulationOptions::fromJson(j.at("simulation"));
    if (j.contains("output")) {
        const auto& o = j.at("output");
        p.outputDirectory = io::resolvePath(o.value("directory", p.outputDirectory), base);
        p.csvInterval = o.value("csvInterval", 0.0);
    } else {
        p.outputDirectory = io::resolvePath(p.outputDirectory, base);
    }
    if (p.name.empty()) p.name = p.rocket.name();
    return p;
}

Project Project::load(const std::string& path) { return fromJson(io::readJsonFile(path), io::directoryOf(path)); }

void Project::save(const std::string& path) const { io::writeJsonFile(path, toJson()); }

Project Project::loadAny(const std::string& path) {
    const json j = io::readJsonFile(path);
    const std::string format = j.value("format", std::string());
    if (format == "rocketup-rocket" || (j.contains("stack") && !j.contains("rocket"))) {
        Project p;
        p.rocket = Rocket::fromJson(j, io::directoryOf(path));
        p.name = p.rocket.name();
        p.outputDirectory = io::resolvePath("output", io::directoryOf(path));
        return p;
    }
    return fromJson(j, io::directoryOf(path));
}

}  // namespace rocketup
