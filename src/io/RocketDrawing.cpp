#include "rocketup/io/RocketDrawing.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rocketup/aero/AeroModel.hpp"

namespace rocketup::io {

namespace {

struct Frame {
    double scale, x0, yMid;
    double X(double x) const { return x0 + x * scale; }
    double Y(double r) const { return yMid - r * scale; }
};

std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '&') o += "&amp;";
        else if (c == '"') o += "&quot;";
        else o.push_back(c);
    }
    return o;
}

}  // namespace

std::string drawRocketSvg(const Rocket& rocket, const DrawingOptions& o) {
    const double L = std::max(rocket.length(), 1e-3);
    double maxR = rocket.maxRadius();
    double maxFin = 0.0;
    for (const auto& p : rocket.flatten())
        if (auto* f = dynamic_cast<const FinSet*>(p.component)) maxFin = std::max(maxFin, f->outerRadius());
    const double extent = std::max(maxR, maxFin);
    const double margin = 40.0;
    const double scale = (o.width - 2 * margin) / L;
    const double height = std::max(2.0 * extent * scale + 2 * margin + (o.showDimensions ? 30.0 : 0.0), 120.0);
    Frame fr{scale, margin, margin + extent * scale + 10.0};

    std::ostringstream s;
    s << std::fixed << std::setprecision(2);
    s << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << o.width << " " << height << "\"";
    if (o.standalone) s << " width=\"" << o.width << "\" height=\"" << height << "\"";
    s << " role=\"img\" aria-label=\"" << esc(rocket.name()) << " side view\" font-family=\"system-ui, sans-serif\">\n";
    s << "<title>" << esc(rocket.name()) << "</title>\n";
    s << "<style>.ru-body{fill:currentColor;fill-opacity:.10;stroke:currentColor;stroke-width:1.4}"
         ".ru-fin{fill:currentColor;fill-opacity:.22;stroke:currentColor;stroke-width:1.2}"
         ".ru-int{fill:none;stroke:currentColor;stroke-opacity:.55;stroke-width:1;stroke-dasharray:4 3}"
         ".ru-motor{fill:#e8590c;fill-opacity:.35;stroke:#e8590c;stroke-width:1.2}"
         ".ru-chute{fill:#2f9e44;fill-opacity:.25;stroke:#2f9e44;stroke-width:1}"
         ".ru-payload{fill:#7048e8;fill-opacity:.25;stroke:#7048e8;stroke-width:1}"
         ".ru-txt{fill:currentColor;font-size:11px}.ru-dim{stroke:currentColor;stroke-opacity:.6;stroke-width:1}</style>\n";
    // Centre line.
    s << "<line class=\"ru-dim\" stroke-dasharray=\"8 4 2 4\" x1=\"" << fr.X(-0.02 * L) << "\" y1=\"" << fr.yMid
      << "\" x2=\"" << fr.X(1.02 * L) << "\" y2=\"" << fr.yMid << "\"/>\n";

    const auto parts = rocket.flatten();
    // ---- body outline
    for (const auto& p : parts) {
        auto* sc = dynamic_cast<const SymmetricComponent*>(p.component);
        if (!sc) continue;
        constexpr int n = 40;
        s << "<polygon class=\"ru-body\" points=\"";
        for (int i = 0; i <= n; ++i) {
            const double x = sc->length() * i / n;
            s << fr.X(p.position + x) << "," << fr.Y(sc->radiusAt(x)) << " ";
        }
        for (int i = n; i >= 0; --i) {
            const double x = sc->length() * i / n;
            s << fr.X(p.position + x) << "," << fr.Y(-sc->radiusAt(x)) << " ";
        }
        s << "\"><title>" << esc(sc->name()) << "</title></polygon>\n";
    }
    // ---- fins (top and bottom views)
    for (const auto& p : parts) {
        auto* f = dynamic_cast<const FinSet*>(p.component);
        if (!f) continue;
        const double rb = f->bodyRadius();
        for (int side : {1, -1}) {
            s << "<polygon class=\"ru-fin\" points=\"";
            for (const auto& pt : f->outline()) s << fr.X(p.position + pt.x) << "," << fr.Y(side * (rb + pt.y)) << " ";
            s << "\"><title>" << esc(f->name()) << " (" << f->count() << " fins)</title></polygon>\n";
        }
    }
    // ---- protrusions
    for (const auto& p : parts) {
        if (auto* l = dynamic_cast<const LaunchLug*>(p.component)) {
            const double r = l->parent() ? l->parent()->outerRadius() : maxR;
            s << "<rect class=\"ru-fin\" x=\"" << fr.X(p.position) << "\" y=\"" << fr.Y(r + 2 * l->outerRadius())
              << "\" width=\"" << l->length() * scale << "\" height=\"" << 2 * l->outerRadius() * scale
              << "\"><title>" << esc(l->name()) << "</title></rect>\n";
        }
    }
    // ---- internals
    if (o.showInternals) {
        for (const auto& p : parts) {
            const Component* c = p.component;
            if (c->isExternal()) continue;
            const double len = c->length();
            double r = c->outerRadius();
            std::string cls = "ru-int";
            if (dynamic_cast<const RecoveryDevice*>(c)) cls = "ru-chute";
            else if (dynamic_cast<const Payload*>(c)) cls = "ru-payload";
            if (r <= 0.0 || len <= 0.0) continue;
            s << "<rect class=\"" << cls << "\" x=\"" << fr.X(p.position) << "\" y=\"" << fr.Y(r) << "\" width=\""
              << len * scale << "\" height=\"" << 2 * r * scale << "\" rx=\"" << (cls == "ru-chute" ? 4 : 1)
              << "\"><title>" << esc(c->name()) << "</title></rect>\n";
            if (auto* mm = dynamic_cast<const MotorMount*>(c); mm && mm->hasMotor()) {
                const Motor& m = mm->motor();
                const double mr = 0.5 * m.diameter;
                s << "<rect class=\"ru-motor\" x=\"" << fr.X(p.position + mm->motorFront()) << "\" y=\"" << fr.Y(mr)
                  << "\" width=\"" << m.length * scale << "\" height=\"" << 2 * mr * scale << "\"><title>"
                  << esc(m.displayName()) << "</title></rect>\n";
            }
        }
    }
    // ---- CG / CP markers
    if (o.showCgCp) {
        const double cg = o.cg >= 0.0 ? o.cg : rocket.massProperties(-1.0).cg;
        double cp = o.cp;
        if (cp < 0.0) {
            AeroModel aero(rocket);
            FlightConditions c;
            c.mach = 0.3;
            c.airspeed = 100.0;
            cp = aero.compute(c, cg).cp;
        }
        const double rm = 9.0;
        const double cx = fr.X(cg), cy = fr.yMid;
        s << "<g><title>CG " << cg << " m</title><circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"" << rm
          << "\" fill=\"#fff\" stroke=\"#111\" stroke-width=\"1.5\"/><path d=\"M" << cx << "," << cy << " L" << cx
          << "," << cy - rm << " A" << rm << "," << rm << " 0 0,1 " << cx + rm << "," << cy << " Z M" << cx << "," << cy
          << " L" << cx << "," << cy + rm << " A" << rm << "," << rm << " 0 0,1 " << cx - rm << "," << cy
          << " Z\" fill=\"#111\"/></g>\n";
        const double px = fr.X(cp);
        s << "<g><title>CP " << cp << " m</title><circle cx=\"" << px << "\" cy=\"" << cy << "\" r=\"" << rm
          << "\" fill=\"#fa5252\" fill-opacity=\".25\" stroke=\"#e03131\" stroke-width=\"1.8\"/><circle cx=\"" << px
          << "\" cy=\"" << cy << "\" r=\"2.6\" fill=\"#e03131\"/></g>\n";
        const double ty = fr.Y(-extent) + 14.0;
        s << "<text class=\"ru-txt\" x=\"" << cx << "\" y=\"" << ty << "\" text-anchor=\"middle\">CG "
          << std::setprecision(3) << cg << " m</text>\n";
        s << "<text class=\"ru-txt\" x=\"" << px << "\" y=\"" << ty + 13.0 << "\" text-anchor=\"middle\" fill=\"#e03131\">CP "
          << cp << " m (" << std::setprecision(2) << (cp - cg) / rocket.referenceDiameter() << " cal)</text>\n";
    }
    if (o.showDimensions) {
        const double y = height - 12.0;
        s << "<line class=\"ru-dim\" x1=\"" << fr.X(0) << "\" y1=\"" << y - 8 << "\" x2=\"" << fr.X(L) << "\" y2=\""
          << y - 8 << "\"/>\n";
        s << "<text class=\"ru-txt\" x=\"" << fr.X(L / 2) << "\" y=\"" << y + 2 << "\" text-anchor=\"middle\">"
          << std::setprecision(3) << L << " m  /  \xC3\x98 " << std::setprecision(0) << rocket.referenceDiameter() * 1000.0
          << " mm</text>\n";
    }
    s << "</svg>\n";
    return s.str();
}

}  // namespace rocketup::io
