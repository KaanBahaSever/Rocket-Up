#include "rocketup/io/Export.hpp"

#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

#include "rocketup/aero/AeroModel.hpp"
#include "rocketup/core/Io.hpp"
#include "rocketup/io/HtmlReport.hpp"

namespace rocketup::io {

std::string slug(const std::string& name) {
    std::string s;
    for (unsigned char c : name) {
        if (std::isalnum(c)) s.push_back(static_cast<char>(std::tolower(c)));
        else if (!s.empty() && s.back() != '_') s.push_back('_');
    }
    while (!s.empty() && s.back() == '_') s.pop_back();
    return s.empty() ? "body" : s;
}

void writeCsv(const BodyResult& body, const std::string& path, const CsvOptions& o) {
    std::vector<const RecordColumn*> cols;
    if (o.columns.empty()) {
        for (const auto& c : recordColumns()) cols.push_back(&c);
    } else {
        for (const auto& k : o.columns) {
            const RecordColumn* c = findRecordColumn(k);
            if (!c) throw RocketUpError("unknown CSV column '" + k + "'");
            cols.push_back(c);
        }
    }
    std::ostringstream ss;
    ss << "# Rocket-Up flight data - body: " << body.name << "\n";
    for (size_t i = 0; i < cols.size(); ++i) {
        if (i) ss << o.delimiter;
        if (o.unitsInHeader) ss << cols[i]->label << " (" << cols[i]->unit << ")";
        else ss << cols[i]->key;
    }
    ss << "\n" << std::setprecision(o.precision);
    double last = -1e300;
    for (size_t r = 0; r < body.records.size(); ++r) {
        const FlightRecord& rec = body.records[r];
        const bool lastRow = r + 1 == body.records.size();
        if (o.interval > 0.0 && rec.time - last < o.interval - 1e-9 && !lastRow) continue;
        last = rec.time;
        for (size_t i = 0; i < cols.size(); ++i) {
            if (i) ss << o.delimiter;
            ss << cols[i]->get(rec);
        }
        ss << "\n";
    }
    writeTextFile(path, ss.str());
}

void writeEventsCsv(const SimulationResult& result, const std::string& path) {
    std::ostringstream ss;
    ss << "Time (s),Event,Body,Source,Altitude AGL (m),Speed (m/s),Message\n" << std::setprecision(7);
    for (const auto& e : result.events)
        ss << e.time << "," << toString(e.type) << ",\"" << e.body << "\",\"" << e.source << "\"," << e.altitude << ","
           << e.speed << ",\"" << e.message << "\"\n";
    writeTextFile(path, ss.str());
}

void writeSummaryJson(const SimulationResult& result, const std::string& path) {
    writeJsonFile(path, result.summaryJson());
}

namespace {
std::string xmlEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        switch (c) {
            case '&': o += "&amp;"; break;
            case '<': o += "&lt;"; break;
            case '>': o += "&gt;"; break;
            case '"': o += "&quot;"; break;
            default: o.push_back(c);
        }
    }
    return o;
}
const char* kmlColors[] = {"ff2f6bff", "ff00c8ff", "ff4caf50", "ffffa000", "ffb04ae0", "ff00e5ff"};
}  // namespace

void writeKml(const SimulationResult& result, const Environment& env, const std::string& path) {
    std::ostringstream k;
    k << std::setprecision(10);
    k << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n";
    k << "<name>" << xmlEscape(result.rocketName) << " - Rocket-Up</name>\n";
    for (size_t i = 0; i < 6; ++i)
        k << "<Style id=\"b" << i << "\"><LineStyle><color>" << kmlColors[i] << "</color><width>3</width></LineStyle>"
          << "<PolyStyle><color>40" << std::string(kmlColors[i]).substr(2) << "</color></PolyStyle></Style>\n";
    k << "<Placemark><name>Launch site</name><Point><coordinates>" << env.site().longitude << "," << env.site().latitude
      << "," << env.site().altitude << "</coordinates></Point></Placemark>\n";
    for (size_t bi = 0; bi < result.bodies.size(); ++bi) {
        const auto& b = result.bodies[bi];
        k << "<Folder><name>" << xmlEscape(b.name) << "</name>\n";
        k << "<Placemark><name>" << xmlEscape(b.name) << " trajectory</name><styleUrl>#b" << bi % 6
          << "</styleUrl><LineString><extrude>1</extrude><altitudeMode>absolute</altitudeMode><coordinates>\n";
        double last = -1e9;
        for (const auto& r : b.records) {
            if (r.time - last < 0.1 && &r != &b.records.back()) continue;
            last = r.time;
            k << r.longitude << "," << r.latitude << "," << r.altitudeAsl << "\n";
        }
        k << "</coordinates></LineString></Placemark>\n";
        for (const auto& e : b.events.all()) {
            if (e.body != b.name || e.type == EventType::Launch) continue;
            double lat, lon;
            // Find the record closest to the event for its position.
            Vec3 pos;
            for (const auto& r : b.records)
                if (r.time >= e.time) {
                    pos = r.position;
                    break;
                }
            env.toGeodetic(pos, lat, lon);
            k << "<Placemark><name>" << toString(e.type) << (e.source.empty() ? "" : " - " + xmlEscape(e.source))
              << "</name><description>t = " << e.time << " s, h = " << e.altitude << " m AGL</description>"
              << "<Point><altitudeMode>absolute</altitudeMode><coordinates>" << lon << "," << lat << ","
              << env.site().altitude + e.altitude << "</coordinates></Point></Placemark>\n";
        }
        if (b.landing.landed)
            k << "<Placemark><name>" << xmlEscape(b.name) << " landing (" << std::setprecision(4) << b.landing.distance
              << " m, " << b.landing.impactSpeed << " m/s)</name><Point><coordinates>" << std::setprecision(10)
              << b.landing.longitude << "," << b.landing.latitude << "," << env.site().altitude
              << "</coordinates></Point></Placemark>\n";
        k << "</Folder>\n";
    }
    k << "</Document>\n</kml>\n";
    writeTextFile(path, k.str());
}

void printSummary(std::ostream& os, const SimulationResult& r) {
    const auto& s = r.summary;
    const auto flags = os.flags();
    os << std::fixed;
    os << "\n=== " << r.rocketName << " - flight summary ===\n";
    os << std::setprecision(1);
    os << "  Apogee               " << std::setw(10) << s.apogee << " m AGL  (" << s.apogeeAsl << " m ASL) at t = "
       << s.apogeeTime << " s\n";
    os << "  Max velocity         " << std::setw(10) << s.maxSpeed << " m/s    (Mach " << std::setprecision(3) << s.maxMach
       << ")\n" << std::setprecision(1);
    os << "  Max acceleration     " << std::setw(10) << s.maxAcceleration << " m/s^2  (" << std::setprecision(2)
       << s.maxAxialG << " g axial)\n" << std::setprecision(1);
    os << "  Max dynamic pressure " << std::setw(10) << s.maxDynamicPressure / 1000.0 << " kPa\n";
    os << "  Rail exit velocity   " << std::setw(10) << s.railExitSpeed << " m/s    at t = " << std::setprecision(2)
       << s.railExitTime << " s\n";
    os << "  Stability            " << std::setw(10) << s.stabilityAtRailExit << " cal at rail exit, min "
       << s.minStability << ", max " << s.maxStability << "\n" << std::setprecision(1);
    os << "  Burnout              " << std::setw(10) << s.burnoutTime << " s      (" << s.burnoutAltitude << " m, "
       << s.burnoutSpeed << " m/s)\n";
    os << "  Max angle of attack  " << std::setw(10) << s.maxAngleOfAttack << " deg\n";
    os << "  Flight time          " << std::setw(10) << s.flightTime << " s\n";
    os << "\n  Bodies:\n";
    for (const auto& b : r.bodies) {
        os << "   - " << b.name << (b.parent.empty() ? "" : " (from " + b.parent + ")") << "\n";
        for (const auto& rec : b.recovery)
            if (rec.deployTime >= 0)
                os << "       " << rec.device << ": deployed t=" << rec.deployTime << " s at " << rec.deployAltitude
                   << " m, " << rec.deploySpeed << " m/s, peak force " << std::setprecision(0) << rec.peakForce
                   << " N\n" << std::setprecision(1);
        if (b.landing.landed)
            os << "       landed t=" << b.landing.time << " s, " << b.landing.distance << " m from pad (bearing "
               << std::setprecision(0) << b.landing.bearing << " deg), impact " << std::setprecision(1)
               << b.landing.impactSpeed << " m/s, descent rate " << b.landing.descentRate << " m/s, KE "
               << std::setprecision(0) << b.landing.kineticEnergy << " J\n" << std::setprecision(1);
    }
    for (const auto& n : s.notes) os << "  ! " << n << "\n";
    for (const auto& w : r.warnings) os << "  ! " << w << "\n";
    os << std::setprecision(3) << "  (" << r.steps << " steps, " << r.computeTime * 1000.0 << " ms)\n";
    os.flags(flags);
}

void printRocketSummary(std::ostream& os, const Rocket& rocket) {
    const auto flags = os.flags();
    const MassProperties loaded = rocket.massProperties(-1.0);
    const MassProperties dry = rocket.dryMassProperties();
    AeroModel aero(rocket);
    os << std::fixed << std::setprecision(3);
    os << "\n=== " << rocket.name() << " ===\n";
    os << "  Length " << rocket.length() << " m, reference diameter " << rocket.referenceDiameter() * 1000.0 << " mm\n";
    os << "  Mass   " << loaded.mass << " kg loaded (" << dry.mass << " kg burnout)\n";
    os << "  CG     " << loaded.cg << " m loaded, " << dry.cg << " m burnout\n";
    FlightConditions c;
    c.mach = 0.3;
    c.airspeed = 100.0;
    const AeroCoefficients a = aero.compute(c, loaded.cg);
    os << "  CP     " << a.cp << " m (Mach 0.3), CNa " << std::setprecision(2) << a.cnAlpha << " /rad\n";
    os << "  Static margin " << (a.cp - loaded.cg) / rocket.referenceDiameter() << " cal loaded, "
       << (a.cp - dry.cg) / rocket.referenceDiameter() << " cal burnout\n";
    os << "  Cd     " << std::setprecision(3) << a.cd << " at Mach 0.3 (friction " << a.cdFriction << ", pressure "
       << a.cdPressure << ", base " << a.cdBase << ")\n";
    for (const auto* m : rocket.motorMounts())
        if (m->hasMotor()) {
            const Motor& mo = m->motor();
            os << "  Motor  " << mo.displayName() << " [" << mo.impulseClass() << "]: " << std::setprecision(0)
               << mo.totalImpulse() << " Ns, " << std::setprecision(2) << mo.burnTime() << " s, Isp "
               << std::setprecision(1) << mo.effectiveIsp() << " s\n";
        }
    for (const auto& issue : rocket.validate()) os << "  ! " << issue << "\n";
    os.flags(flags);
}

std::vector<std::string> exportAll(const SimulationResult& result, const Rocket& rocket, const Environment& env,
                                   const std::string& dir) {
    std::vector<std::string> files;
    std::filesystem::create_directories(std::filesystem::u8path(dir));
    auto p = [&](const std::string& f) { return (std::filesystem::u8path(dir) / std::filesystem::u8path(f)).u8string(); };
    for (const auto& b : result.bodies) {
        const std::string f = p("flight_" + slug(b.name) + ".csv");
        writeCsv(b, f);
        files.push_back(f);
    }
    writeEventsCsv(result, p("events.csv"));
    files.push_back(p("events.csv"));
    writeSummaryJson(result, p("summary.json"));
    files.push_back(p("summary.json"));
    writeKml(result, env, p("trajectory.kml"));
    files.push_back(p("trajectory.kml"));
    writeHtmlReport(result, rocket, env, p("report.html"));
    files.push_back(p("report.html"));
    return files;
}

}  // namespace rocketup::io
