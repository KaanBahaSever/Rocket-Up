#include <cmath>
#include <map>
#include <nlohmann/json.hpp>
#include <regex>
#include <sstream>

#include "rocketup/core/Io.hpp"
#include "rocketup/propulsion/Motor.hpp"

namespace rocketup {

namespace {

std::string expandManufacturer(const std::string& abbr) {
    const std::string a = io::toLower(abbr);
    if (a == "cti" || a == "cesaroni") return "Cesaroni Technology";
    if (a == "at" || a == "aerotech" || a == "aerotech-rcs") return "AeroTech";
    if (a == "es" || a == "estes") return "Estes Industries";
    if (a == "loki") return "Loki Research";
    if (a == "ae" || a == "animal") return "Animal Motor Works";
    if (a == "kba") return "Kosdon by AeroTech";
    if (a == "ct") return "Contrail Rockets";
    if (a == "hs" || a == "hypertek") return "Hypertek";
    if (a == "q" || a == "quest") return "Quest Aerospace";
    return abbr;
}

std::map<std::string, std::string> xmlAttributes(const std::string& tag) {
    std::map<std::string, std::string> out;
    static const std::regex attr(R"re(([A-Za-z_][\w\-]*)\s*=\s*"([^"]*)")re");
    for (auto it = std::sregex_iterator(tag.begin(), tag.end(), attr); it != std::sregex_iterator(); ++it)
        out[(*it)[1].str()] = (*it)[2].str();
    return out;
}

double attrDouble(const std::map<std::string, std::string>& a, const std::string& key, double fallback = 0.0) {
    auto it = a.find(key);
    double v;
    if (it == a.end() || !io::tryParseDouble(it->second, v)) return fallback;
    return v;
}

void applyOptions(Motor& m, const MotorLoadOptions& o) {
    if (o.designation) m.designation = *o.designation;
    if (o.manufacturer) m.manufacturer = *o.manufacturer;
    if (o.propellantMass) m.propellantMass = *o.propellantMass;
    if (o.totalMass) m.totalMass = *o.totalMass;
    if (o.isp) m.isp = *o.isp;
    if (o.diameter) m.diameter = *o.diameter;
    if (o.length) m.length = *o.length;
    if (o.nozzleExitDiameter) m.nozzleExitArea = mathrix::kPi * *o.nozzleExitDiameter * *o.nozzleExitDiameter / 4.0;
    if (o.interpolation) m.interpolation = *o.interpolation;
    if (o.massModel) m.massModel = *o.massModel;
}

}  // namespace

std::vector<Motor> MotorLoader::parseEng(const std::string& text) {
    std::vector<Motor> motors;
    std::istringstream in(text);
    std::string line;
    std::string pendingComment;
    Motor* cur = nullptr;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string t = io::trim(line);
        if (t.empty()) continue;
        if (t[0] == ';') {
            if (!cur || cur->time.empty()) pendingComment += io::trim(t.substr(1)) + " ";
            continue;
        }
        const auto tok = io::splitWhitespace(t);
        double a, b;
        if (tok.size() >= 2 && io::tryParseDouble(tok[0], a) && io::tryParseDouble(tok[1], b)) {
            if (!cur) throw RocketUpError("RASP file: data before header line");
            for (size_t i = 0; i + 1 < tok.size(); i += 2) {
                if (!io::tryParseDouble(tok[i], a) || !io::tryParseDouble(tok[i + 1], b)) break;
                cur->time.push_back(a);
                cur->thrust.push_back(b);
            }
            continue;
        }
        if (tok.size() < 7) throw RocketUpError("RASP file: malformed header '" + t + "'");
        motors.emplace_back();
        cur = &motors.back();
        cur->designation = tok[0];
        double v;
        if (io::tryParseDouble(tok[1], v)) cur->diameter = v / 1000.0;
        if (io::tryParseDouble(tok[2], v)) cur->length = v / 1000.0;
        for (auto& d : io::split(tok[3], '-'))
            if (!d.empty()) cur->delays.push_back(d);
        if (io::tryParseDouble(tok[4], v)) cur->propellantMass = v;
        if (io::tryParseDouble(tok[5], v)) cur->totalMass = v;
        std::string mfg;
        for (size_t i = 6; i < tok.size(); ++i) mfg += (i > 6 ? " " : "") + tok[i];
        cur->manufacturer = expandManufacturer(mfg);
        cur->source.importedFrom = "RASP .eng";
        cur->source.notes = io::trim(pendingComment);
        pendingComment.clear();
    }
    if (motors.empty()) throw RocketUpError("RASP file contains no motor");
    return motors;
}

std::vector<Motor> MotorLoader::parseRse(const std::string& text) {
    std::vector<Motor> motors;
    static const std::regex engineRe(R"re(<engine\s([^>]*)>([\s\S]*?)</engine>)re");
    static const std::regex dataRe(R"re(<eng-data\s([^>]*)/?>)re");
    static const std::regex commentRe(R"re(<comments>([\s\S]*?)</comments>)re");
    for (auto it = std::sregex_iterator(text.begin(), text.end(), engineRe); it != std::sregex_iterator(); ++it) {
        const auto a = xmlAttributes((*it)[1].str());
        const std::string body = (*it)[2].str();
        Motor m;
        m.designation = a.count("code") ? a.at("code") : "unknown";
        m.manufacturer = expandManufacturer(a.count("mfg") ? a.at("mfg") : "");
        m.diameter = attrDouble(a, "dia") / 1000.0;
        m.length = attrDouble(a, "len") / 1000.0;
        m.totalMass = attrDouble(a, "initWt") / 1000.0;
        m.propellantMass = attrDouble(a, "propWt") / 1000.0;
        m.isp = attrDouble(a, "Isp");
        const double exitDia = attrDouble(a, "exitDia") / 1000.0;
        if (exitDia > 0.0) m.nozzleExitArea = mathrix::kPi * exitDia * exitDia / 4.0;
        if (a.count("Type")) {
            const std::string ty = io::toLower(a.at("Type"));
            m.motorType = ty == "single-use" ? "single-use" : (ty == "hybrid" ? "hybrid" : "reload");
        }
        if (a.count("delays"))
            for (auto& d : io::split(a.at("delays"), ','))
                if (!d.empty()) m.delays.push_back(d);
        std::smatch cm;
        if (std::regex_search(body, cm, commentRe)) m.source.notes = io::trim(cm[1].str());
        bool hasMass = false, hasCg = false;
        double cg = 0.0;
        for (auto d = std::sregex_iterator(body.begin(), body.end(), dataRe); d != std::sregex_iterator(); ++d) {
            const auto da = xmlAttributes((*d)[1].str());
            m.time.push_back(attrDouble(da, "t"));
            m.thrust.push_back(attrDouble(da, "f"));
            if (da.count("m")) {
                hasMass = true;
                m.massTime.push_back(attrDouble(da, "t"));
                m.massPropellant.push_back(attrDouble(da, "m") / 1000.0);
            }
            if (da.count("cg")) {
                hasCg = true;
                cg = attrDouble(da, "cg") / 1000.0;
            }
        }
        if (!hasMass) {
            m.massTime.clear();
            m.massPropellant.clear();
        }
        if (hasCg && cg > 0.0) m.cgFromTop = cg;
        m.source.importedFrom = "RockSim .rse";
        motors.push_back(std::move(m));
    }
    if (motors.empty()) throw RocketUpError("RSE file contains no <engine> element");
    return motors;
}

Motor MotorLoader::parseCsv(const std::string& text) {
    const auto t = io::parseCsv(text);
    if (t.columnCount() < 2 || t.rows.empty()) throw RocketUpError("thrust CSV needs time and thrust columns");
    Motor m;
    for (const auto& c : t.comments) {
        auto f = io::split(c, ',');
        if (f.size() < 2) continue;
        const std::string key = io::toLower(f[0]);
        if (key == "motor:") {
            m.designation = f[1];
            // "Cesaroni 8429M2020-P" -> manufacturer + designation
            const auto words = io::splitWhitespace(f[1]);
            if (words.size() >= 2) {
                m.manufacturer = expandManufacturer(words[0]);
                m.designation = words.back();
            }
        } else if (key == "contributor:") m.source.contributor = f[1];
        else if (key == "details:" || key == "url:") m.source.url = f[1];
    }
    int tcol = t.findColumn("time");
    int fcol = t.findColumn("thrust");
    if (tcol < 0) tcol = 0;
    if (fcol < 0) fcol = 1;
    m.time = t.column(static_cast<size_t>(tcol));
    m.thrust = t.column(static_cast<size_t>(fcol));
    const int mcol = t.findColumn("mass");
    if (mcol >= 0) {
        m.massTime = m.time;
        m.massPropellant = t.column(static_cast<size_t>(mcol));
        const std::string h = io::toLower(t.header[static_cast<size_t>(mcol)]);
        if (h.find("(g)") != std::string::npos)
            for (auto& v : m.massPropellant) v /= 1000.0;
    }
    m.source.importedFrom = "CSV";
    return m;
}

std::vector<Motor> MotorLoader::loadAll(const std::string& path, const MotorLoadOptions& options) {
    const std::string ext = io::extensionOf(path);
    std::vector<Motor> motors;
    if (ext == ".rumotor" || ext == ".json") {
        Motor m = Motor::fromJson(io::readJsonFile(path));
        motors.push_back(std::move(m));
    } else {
        const std::string text = io::readTextFile(path);
        if (ext == ".eng") motors = parseEng(text);
        else if (ext == ".rse" || text.find("<engine-database") != std::string::npos) motors = parseRse(text);
        else motors.push_back(parseCsv(text));
    }
    for (auto& m : motors) {
        if (m.source.importedFrom.empty() || m.source.importedFrom == "CSV") {
            const auto slash = path.find_last_of("/\\");
            const std::string fname = slash == std::string::npos ? path : path.substr(slash + 1);
            if (m.designation.empty()) m.designation = fname.substr(0, fname.find('.'));
        }
        applyOptions(m, options);
        m.finalize();
    }
    return motors;
}

Motor MotorLoader::load(const std::string& path, const MotorLoadOptions& options) {
    auto all = loadAll(path, options);
    return std::move(all.front());
}

void MotorLoader::save(const Motor& motor, const std::string& path) { io::writeJsonFile(path, motor.toJson()); }

}  // namespace rocketup
