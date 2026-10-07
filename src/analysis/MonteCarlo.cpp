#include "rocketup/analysis/MonteCarlo.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <thread>

#include "rocketup/core/Io.hpp"
#include "rocketup/io/HtmlReport.hpp"

namespace rocketup {

json Dispersion::toJson() const {
    return {{"windSpeedFactor", windSpeedFactor}, {"windDirection", windDirection}, {"windExtra", windExtra},
            {"thrust", thrust},                   {"dragCoefficient", dragCoefficient}, {"mass", mass},
            {"railElevation", railElevation},     {"railAzimuth", railAzimuth},       {"parachuteCd", parachuteCd}};
}

Dispersion Dispersion::fromJson(const json& j) {
    Dispersion d;
    d.windSpeedFactor = j.value("windSpeedFactor", d.windSpeedFactor);
    d.windDirection = j.value("windDirection", d.windDirection);
    d.windExtra = j.value("windExtra", d.windExtra);
    d.thrust = j.value("thrust", d.thrust);
    d.dragCoefficient = j.value("dragCoefficient", d.dragCoefficient);
    d.mass = j.value("mass", d.mass);
    d.railElevation = j.value("railElevation", d.railElevation);
    d.railAzimuth = j.value("railAzimuth", d.railAzimuth);
    d.parachuteCd = j.value("parachuteCd", d.parachuteCd);
    return d;
}

json MonteCarloResult::toJson() const {
    json runsJ = json::array();
    for (const auto& r : runs) {
        json land = json::object();
        for (const auto& [k, v] : r.landing) land[k] = {v.x, v.y};
        runsJ.push_back({{"apogee", r.apogee}, {"maxSpeed", r.maxSpeed}, {"railExitSpeed", r.railExitSpeed},
                         {"minStability", r.minStability}, {"thrustScale", r.thrustScale}, {"cdScale", r.cdScale},
                         {"windScale", r.windScale}, {"windRotation", r.windRotation}, {"landing", land}});
    }
    json lands = json::array();
    for (const auto& l : landings)
        lands.push_back({{"body", l.body},
                         {"mean", {l.mean.x, l.mean.y}},
                         {"covariance", {{l.covEE, l.covEN}, {l.covEN, l.covNN}}},
                         {"semiMajor1Sigma", l.semiMajor},
                         {"semiMinor1Sigma", l.semiMinor},
                         {"angleDeg", l.angle},
                         {"maxDistance", l.maxDistance},
                         {"meanImpactSpeed", l.meanImpactSpeed}});
    return {{"apogee", {{"mean", apogeeMean}, {"std", apogeeStd}, {"min", apogeeMin}, {"max", apogeeMax}}},
            {"landings", lands},
            {"runs", runsJ},
            {"computeTime", computeTime}};
}

namespace {

struct RunOutput {
    MonteCarloRun run;
    std::map<std::string, double> impact;
};

RunOutput runOne(const Rocket& base, const Environment& baseEnv, const MonteCarloOptions& o, int index) {
    std::mt19937_64 rng(o.seed + 104729ULL * static_cast<uint64_t>(index));
    std::normal_distribution<double> N(0.0, 1.0);
    const Dispersion& d = o.dispersion;
    RunOutput out;
    Rocket r = base;
    out.run.cdScale = 1.0 + d.dragCoefficient * N(rng);
    r.aero().cdMultiplier *= out.run.cdScale;
    out.run.thrustScale = 1.0 + d.thrust * N(rng);
    std::vector<std::string> mountNames, chuteNames;
    for (const auto& p : r.flatten()) {
        if (dynamic_cast<const MotorMount*>(p.component)) mountNames.push_back(p.component->name());
        if (dynamic_cast<const Parachute*>(p.component)) chuteNames.push_back(p.component->name());
    }
    for (const auto& n : mountNames) {
        auto* mm = r.findAs<MotorMount>(n);
        if (mm && mm->hasMotor()) {
            Motor m = mm->motor();
            m.thrustScale *= out.run.thrustScale;
            mm->setMotor(m);
        }
    }
    for (const auto& n : chuteNames)
        if (auto* pc = r.findAs<Parachute>(n)) pc->setCd(pc->cd() * (1.0 + d.parachuteCd * N(rng)));
    const MassProperties s = r.structureMass();
    r.overrideDryMass(s.mass * (1.0 + d.mass * N(rng)), s.cg);

    Environment env = baseEnv;
    LaunchRail rail = env.rail();
    rail.elevation = std::clamp(rail.elevation + d.railElevation * N(rng), 1.0, 90.0);
    rail.azimuth += d.railAzimuth * N(rng);
    env.setRail(rail);
    out.run.windScale = std::max(0.0, 1.0 + d.windSpeedFactor * N(rng));
    out.run.windRotation = d.windDirection * N(rng);
    const double extraDir = std::uniform_real_distribution<double>(0.0, mathrix::kTwoPi)(rng);
    const double extra = std::abs(d.windExtra * N(rng));
    env.setWind(std::make_unique<TransformedWind>(baseEnv.wind().clone(), out.run.windScale, out.run.windRotation,
                                                  Vec3{extra * std::sin(extraDir), extra * std::cos(extraDir), 0.0}));
    SimulationOptions so = o.simulation;
    so.seed = o.seed + static_cast<uint64_t>(index);
    const SimulationResult res = simulate(r, env, so);
    out.run.apogee = res.summary.apogee;
    out.run.maxSpeed = res.summary.maxSpeed;
    out.run.railExitSpeed = res.summary.railExitSpeed;
    out.run.minStability = res.summary.minStability;
    for (const auto& b : res.bodies)
        if (b.landing.landed) {
            out.run.landing[b.name] = b.landing.position;
            out.impact[b.name] = b.landing.impactSpeed;
        }
    return out;
}

}  // namespace

MonteCarloResult runMonteCarlo(const Rocket& rocket, const Environment& env, const MonteCarloOptions& o,
                               const std::function<void(int, int)>& progress) {
    const auto t0 = std::chrono::steady_clock::now();
    const int n = std::max(1, o.runs);
    int threads = o.threads > 0 ? o.threads : static_cast<int>(std::thread::hardware_concurrency());
    threads = std::clamp(threads, 1, n);
    std::vector<RunOutput> outputs(static_cast<size_t>(n));
    std::atomic<int> next{0}, done{0};
    std::mutex progressMutex;
    std::vector<std::string> errors;
    auto worker = [&] {
        for (;;) {
            const int i = next++;
            if (i >= n) break;
            try {
                outputs[static_cast<size_t>(i)] = runOne(rocket, env, o, i);
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(progressMutex);
                errors.push_back(e.what());
            }
            const int d = ++done;
            if (progress) {
                std::lock_guard<std::mutex> lock(progressMutex);
                progress(d, n);
            }
        }
    };
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; ++t) pool.emplace_back(worker);
    for (auto& t : pool) t.join();
    if (!errors.empty()) throw RocketUpError("Monte Carlo run failed: " + errors.front());

    MonteCarloResult r;
    std::map<std::string, std::vector<std::pair<Vec3, double>>> pts;
    std::vector<std::string> order;
    for (auto& out : outputs) {
        r.runs.push_back(out.run);
        for (const auto& [name, p] : out.run.landing) {
            if (!pts.count(name)) order.push_back(name);
            pts[name].push_back({p, out.impact[name]});
        }
    }
    double sum = 0, sum2 = 0;
    r.apogeeMin = 1e300;
    r.apogeeMax = -1e300;
    for (const auto& run : r.runs) {
        sum += run.apogee;
        sum2 += run.apogee * run.apogee;
        r.apogeeMin = std::min(r.apogeeMin, run.apogee);
        r.apogeeMax = std::max(r.apogeeMax, run.apogee);
    }
    r.apogeeMean = sum / n;
    r.apogeeStd = std::sqrt(std::max(0.0, sum2 / n - r.apogeeMean * r.apogeeMean));
    for (const auto& name : order) {
        LandingStatistics s;
        s.body = name;
        const auto& v = pts[name];
        for (const auto& [p, imp] : v) {
            s.points.push_back(p);
            s.mean += p;
            s.meanImpactSpeed += imp;
            s.maxDistance = std::max(s.maxDistance, std::hypot(p.x, p.y));
        }
        const double m = static_cast<double>(v.size());
        s.mean /= m;
        s.meanImpactSpeed /= m;
        for (const auto& [p, imp] : v) {
            const double dx = p.x - s.mean.x, dy = p.y - s.mean.y;
            s.covEE += dx * dx / m;
            s.covNN += dy * dy / m;
            s.covEN += dx * dy / m;
        }
        const double tr = 0.5 * (s.covEE + s.covNN);
        const double det = std::sqrt(std::max(0.0, 0.25 * (s.covEE - s.covNN) * (s.covEE - s.covNN) + s.covEN * s.covEN));
        s.semiMajor = std::sqrt(std::max(0.0, tr + det));
        s.semiMinor = std::sqrt(std::max(0.0, tr - det));
        s.angle = mathrix::rad2deg(0.5 * std::atan2(2.0 * s.covEN, s.covEE - s.covNN));
        r.landings.push_back(std::move(s));
    }
    r.computeTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

namespace {
std::vector<std::pair<double, double>> ellipse(const LandingStatistics& s, double k) {
    std::vector<std::pair<double, double>> out;
    const double a = mathrix::deg2rad(s.angle);
    for (int i = 0; i <= 72; ++i) {
        const double t = mathrix::kTwoPi * i / 72.0;
        const double x = k * s.semiMajor * std::cos(t), y = k * s.semiMinor * std::sin(t);
        out.emplace_back(s.mean.x + x * std::cos(a) - y * std::sin(a), s.mean.y + x * std::sin(a) + y * std::cos(a));
    }
    return out;
}
}  // namespace

void writeMonteCarloKml(const MonteCarloResult& r, const Environment& env, const std::string& path) {
    std::ostringstream k;
    k << std::setprecision(10);
    k << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>\n"
      << "<name>Rocket-Up Monte Carlo landing dispersion</name>\n"
      << "<Style id=\"pt\"><IconStyle><scale>0.4</scale></IconStyle></Style>\n"
      << "<Style id=\"el\"><LineStyle><color>ff0080ff</color><width>2</width></LineStyle>"
         "<PolyStyle><color>300080ff</color></PolyStyle></Style>\n";
    for (const auto& s : r.landings) {
        k << "<Folder><name>" << s.body << "</name>\n";
        for (int sig : {1, 2, 3}) {
            k << "<Placemark><name>" << s.body << " " << sig << " sigma</name><styleUrl>#el</styleUrl><Polygon>"
              << "<outerBoundaryIs><LinearRing><coordinates>\n";
            for (auto& [x, y] : ellipse(s, sig)) {
                double lat, lon;
                env.toGeodetic({x, y, 0.0}, lat, lon);
                k << lon << "," << lat << ",0\n";
            }
            k << "</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>\n";
        }
        for (const auto& p : s.points) {
            double lat, lon;
            env.toGeodetic(p, lat, lon);
            k << "<Placemark><styleUrl>#pt</styleUrl><Point><coordinates>" << lon << "," << lat
              << ",0</coordinates></Point></Placemark>\n";
        }
        k << "</Folder>\n";
    }
    k << "</Document></kml>\n";
    io::writeTextFile(path, k.str());
}

void writeMonteCarloReport(const MonteCarloResult& r, const Rocket& rocket, const Environment& env,
                           const std::string& path) {
    json data;
    data["rocket"] = rocket.name();
    data["site"] = env.site().name;
    data["apogee"] = {{"mean", r.apogeeMean}, {"std", r.apogeeStd}, {"min", r.apogeeMin}, {"max", r.apogeeMax}};
    json ap = json::array();
    for (const auto& run : r.runs) ap.push_back(run.apogee);
    data["apogees"] = ap;
    json lands = json::array();
    for (const auto& s : r.landings) {
        json e, nn;
        for (const auto& p : s.points) {
            e.push_back(p.x);
            nn.push_back(p.y);
        }
        json ells = json::array();
        for (int sig : {1, 2, 3}) {
            json ex, ey;
            for (auto& [x, y] : ellipse(s, sig)) {
                ex.push_back(x);
                ey.push_back(y);
            }
            ells.push_back({{"x", ex}, {"y", ey}});
        }
        lands.push_back({{"body", s.body}, {"e", e}, {"n", nn}, {"mean", {s.mean.x, s.mean.y}}, {"a", s.semiMajor},
                         {"b", s.semiMinor}, {"angle", s.angle}, {"max", s.maxDistance},
                         {"impact", s.meanImpactSpeed}, {"ellipses", ells}});
    }
    data["landings"] = lands;
    data["runs"] = r.runs.size();
    data["time"] = r.computeTime;
    std::string html = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Monte Carlo dispersion</title>
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>:root{--bg:#f6f7fb;--card:#fff;--ink:#1b1f3b;--muted:#5c6378;--line:#e3e6ef;--grid:#e9ecf2}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#0d1124;--card:#151a33;--ink:#e9ecf5;--muted:#9aa3bd;--line:#252c4d;--grid:#232a48}}
:root[data-theme="dark"]{--bg:#0d1124;--card:#151a33;--ink:#e9ecf5;--muted:#9aa3bd;--line:#252c4d;--grid:#232a48}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,sans-serif}
header{display:flex;gap:16px;align-items:center;padding:18px 24px;background:linear-gradient(120deg,#141c48,#26387f);color:#fff}
header .logo{width:52px;height:52px}header svg{width:100%;height:100%}h1{margin:0;font-size:22px}
main{max-width:1200px;margin:0 auto;padding:20px 16px}.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:16px}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}@media(max-width:900px){.grid{grid-template-columns:1fr}}
.chart{height:460px}table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;font-size:14px}th,td{padding:6px 8px;border-bottom:1px solid var(--line);text-align:left}th{color:var(--muted)}
.tbl{overflow-x:auto}</style></head><body>
<header><div class="logo">{{LOGO}}</div><div><h1 id="t"></h1><div id="s" style="opacity:.8"></div></div></header>
<main><section class="card"><div class="tbl"><table id="tab"></table></div></section>
<div class="grid"><section class="card"><h3>Landing dispersion (1, 2, 3 sigma)</h3><div class="chart" id="map"></div></section>
<section class="card"><h3>Apogee distribution</h3><div class="chart" id="hist"></div></section></div></main>
<script>const D={{DATA}};const C=['#ff6b35','#1c7ed6','#2f9e44','#ae3ec9','#f59f00'];
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();const f=(v,d=0)=>Number(v).toLocaleString('en-US',{maximumFractionDigits:d,minimumFractionDigits:d});
document.getElementById('t').textContent=D.rocket+' - Monte Carlo dispersion';
document.getElementById('s').textContent=`${D.runs} runs in ${f(D.time,1)} s · apogee ${f(D.apogee.mean)} ± ${f(D.apogee.std)} m (min ${f(D.apogee.min)}, max ${f(D.apogee.max)})`;
document.getElementById('tab').innerHTML='<tr><th>Body</th><th>Mean landing (E, N)</th><th>1σ ellipse</th><th>Orientation</th><th>Max distance</th><th>Mean impact</th></tr>'+D.landings.map(l=>`<tr><td>${l.body}</td><td>${f(l.mean[0])} m, ${f(l.mean[1])} m</td><td>${f(l.a)} × ${f(l.b)} m</td><td>${f(l.angle)}°</td><td>${f(l.max)} m</td><td>${f(l.impact,1)} m/s</td></tr>`).join('');
const L={paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{color:css('--ink')},margin:{l:60,r:20,t:10,b:50},legend:{orientation:'h',y:-0.15}};
const tr=[{x:[0],y:[0],mode:'markers',name:'pad',marker:{symbol:'triangle-up',size:13,color:css('--ink')}}];
D.landings.forEach((l,i)=>{tr.push({x:l.e,y:l.n,mode:'markers',name:l.body,marker:{size:5,color:C[i%5],opacity:.6}});l.ellipses.forEach((e,k)=>tr.push({x:e.x,y:e.y,mode:'lines',showlegend:false,line:{color:C[i%5],width:2-k*0.4,dash:['solid','dash','dot'][k]},hoverinfo:'skip'}));});
Plotly.newPlot('map',tr,Object.assign({},L,{xaxis:{title:'East (m)',gridcolor:css('--grid')},yaxis:{title:'North (m)',scaleanchor:'x',gridcolor:css('--grid')}}),{responsive:true,displaylogo:false});
Plotly.newPlot('hist',[{x:D.apogees,type:'histogram',marker:{color:'#ff6b35'}}],Object.assign({},L,{xaxis:{title:'Apogee (m)',gridcolor:css('--grid')},yaxis:{title:'Runs',gridcolor:css('--grid')}}),{responsive:true,displaylogo:false});
</script></body></html>)HTML";
    auto rep = [&](const std::string& a, const std::string& b) {
        const size_t p = html.find(a);
        if (p != std::string::npos) html.replace(p, a.size(), b);
    };
    rep("{{LOGO}}", io::logoSvg());
    rep("{{DATA}}", data.dump());
    io::writeTextFile(path, html);
}

}  // namespace rocketup
