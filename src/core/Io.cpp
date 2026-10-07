#include "rocketup/core/Io.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace fs = std::filesystem;

namespace rocketup::io {

std::string readTextFile(const std::string& path) {
    std::ifstream in(fs::u8path(path), std::ios::binary);
    if (!in) throw RocketUpError("cannot open file '" + path + "'");
    std::ostringstream ss;
    ss << in.rdbuf();
    std::string s = ss.str();
    // Strip UTF-8 BOM.
    if (s.size() >= 3 && static_cast<unsigned char>(s[0]) == 0xEF && static_cast<unsigned char>(s[1]) == 0xBB &&
        static_cast<unsigned char>(s[2]) == 0xBF)
        s.erase(0, 3);
    return s;
}

void writeTextFile(const std::string& path, const std::string& content) {
    const fs::path p = fs::u8path(path);
    if (p.has_parent_path()) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
    }
    std::ofstream out(p, std::ios::binary);
    if (!out) throw RocketUpError("cannot write file '" + path + "'");
    out << content;
    if (!out) throw RocketUpError("error while writing file '" + path + "'");
}

json readJsonFile(const std::string& path) {
    const std::string text = readTextFile(path);
    try {
        return json::parse(text, nullptr, true, /*ignore_comments=*/true);
    } catch (const json::exception& e) {
        throw RocketUpError("invalid JSON in '" + path + "': " + e.what());
    }
}

void writeJsonFile(const std::string& path, const json& j, int indent) { writeTextFile(path, j.dump(indent) + "\n"); }

std::string resolvePath(const std::string& path, const std::string& baseDirectory) {
    if (path.empty()) return path;
    const fs::path p = fs::u8path(path);
    if (p.is_absolute() || baseDirectory.empty()) return path;
    return (fs::u8path(baseDirectory) / p).lexically_normal().u8string();
}

std::string directoryOf(const std::string& path) {
    const fs::path p = fs::u8path(path);
    return p.has_parent_path() ? p.parent_path().u8string() : std::string();
}

std::string extensionOf(const std::string& path) { return toLower(fs::u8path(path).extension().u8string()); }

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;
    for (char c : s) {
        if (c == '"') {
            quoted = !quoted;
            continue;
        }
        if (c == delimiter && !quoted) {
            out.push_back(trim(cur));
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    out.push_back(trim(cur));
    return out;
}

std::vector<std::string> splitWhitespace(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string tok;
    while (ss >> tok) out.push_back(tok);
    return out;
}

bool tryParseDouble(const std::string& text, double& out) {
    std::string t = trim(text);
    if (t.empty()) return false;
    std::replace(t.begin(), t.end(), ',', '.');
    char* end = nullptr;
    out = std::strtod(t.c_str(), &end);
    return end != t.c_str() && *end == '\0';
}

std::vector<double> CsvTable::column(size_t index) const {
    std::vector<double> c;
    c.reserve(rows.size());
    for (auto& r : rows)
        if (index < r.size()) c.push_back(r[index]);
    return c;
}

int CsvTable::findColumn(const std::string& needle) const {
    const std::string n = toLower(needle);
    for (size_t i = 0; i < header.size(); ++i)
        if (toLower(header[i]).find(n) != std::string::npos) return static_cast<int>(i);
    return -1;
}

CsvTable parseCsv(const std::string& text) {
    CsvTable t;
    std::istringstream in(text);
    std::string line;
    // Guess delimiter: ';' files with decimal commas are common in Europe.
    const bool semicolon = std::count(text.begin(), text.end(), ';') > std::count(text.begin(), text.end(), '\n') / 2 &&
                           text.find(";") != std::string::npos;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string tl = trim(line);
        if (tl.empty()) continue;
        if (tl[0] == '#' || (tl[0] == ';' && !semicolon)) {
            t.comments.push_back(tl);
            continue;
        }
        std::vector<std::string> fields;
        if (semicolon) fields = split(tl, ';');
        else if (tl.find(',') != std::string::npos) fields = split(tl, ',');
        else if (tl.find('\t') != std::string::npos) fields = split(tl, '\t');
        else fields = splitWhitespace(tl);
        while (!fields.empty() && fields.back().empty()) fields.pop_back();
        std::vector<double> row;
        bool numeric = !fields.empty();
        for (auto& f : fields) {
            double v;
            if (!tryParseDouble(f, v)) {
                numeric = false;
                break;
            }
            row.push_back(v);
        }
        if (numeric) {
            t.rows.push_back(std::move(row));
        } else if (t.rows.empty() && fields.size() >= 2 && t.header.empty()) {
            // Could be a header ("Time (s)","Thrust (N)" / "mach,0,3000") or metadata
            // ("motor:","X"); metadata lines have a trailing colon on the first field.
            if (!fields.front().empty() && fields.front().back() == ':') t.comments.push_back(tl);
            else t.header = fields;
        } else {
            t.comments.push_back(tl);
        }
    }
    return t;
}

CsvTable readCsvFile(const std::string& path) { return parseCsv(readTextFile(path)); }

}  // namespace rocketup::io
