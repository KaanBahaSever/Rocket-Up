#pragma once

#include <nlohmann/json.hpp>
#include <string>

#include "rocketup/core/Types.hpp"

namespace rocketup::detail {

/// Directory of the design file currently being loaded (for relative motor/table paths).
inline std::string& currentBaseDirectory() {
    thread_local std::string dir;
    return dir;
}

/// RAII guard setting `currentBaseDirectory()` while a file is parsed.
class BaseDirectoryScope {
public:
    explicit BaseDirectoryScope(std::string dir) : previous_(currentBaseDirectory()) {
        currentBaseDirectory() = std::move(dir);
    }
    ~BaseDirectoryScope() { currentBaseDirectory() = previous_; }
    BaseDirectoryScope(const BaseDirectoryScope&) = delete;
    BaseDirectoryScope& operator=(const BaseDirectoryScope&) = delete;

private:
    std::string previous_;
};

/// Read a radius given either as `<radiusKey>` or `<diameterKey>` (halved).
inline double readRadius(const json& j, const char* radiusKey, const char* diameterKey, double fallback) {
    if (j.contains(radiusKey)) return j.at(radiusKey).get<double>();
    if (j.contains(diameterKey)) return 0.5 * j.at(diameterKey).get<double>();
    return fallback;
}

}  // namespace rocketup::detail
