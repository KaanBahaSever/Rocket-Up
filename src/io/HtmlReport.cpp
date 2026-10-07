#include "rocketup/io/HtmlReport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <nlohmann/json.hpp>

#include "rocketup/Logo.hpp"
#include "rocketup/aero/AeroModel.hpp"
#include "rocketup/core/Io.hpp"
#include "rocketup/io/RocketDrawing.hpp"

namespace rocketup::io {

const std::string& logoSvg() {
    static const std::string s = generated::kLogoSvg;
    return s;
}

namespace {

double r6(double v) {
    if (!std::isfinite(v)) return 0.0;
    char b[32];
    std::snprintf(b, sizeof b, "%.6g", v);
    return std::atof(b);
}

/// Indices to keep: dense during ascent, sparser during descent, always the last sample.
std::vector<size_t> downsample(const std::vector<FlightRecord>& rec, int maxPoints) {
    std::vector<size_t> asc, desc, out;
    for (size_t i = 0; i < rec.size(); ++i)
        (rec[i].phase == FlightPhase::Descent || rec[i].phase == FlightPhase::Landed ? desc : asc).push_back(i);
    const size_t budgetAsc = static_cast<size_t>(maxPoints * 0.7), budgetDesc = static_cast<size_t>(maxPoints * 0.3);
    auto take = [&](const std::vector<size_t>& v, size_t budget) {
        if (v.empty()) return;
        const size_t stride = std::max<size_t>(1, (v.size() + budget - 1) / std::max<size_t>(budget, 1));
        for (size_t i = 0; i < v.size(); i += stride) out.push_back(v[i]);
        if (out.back() != v.back()) out.push_back(v.back());
    };
    take(asc, budgetAsc);
    take(desc, budgetDesc);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

json seriesJson(const std::vector<FlightRecord>& records, int maxPoints) {
    const auto idx = downsample(records, maxPoints);
    std::map<std::string, std::vector<double>> s;
    auto put = [&](const char* k, double v) { s[k].push_back(r6(v)); };
    for (size_t i : idx) {
        const FlightRecord& r = records[i];
        put("t", r.time);
        put("alt", r.altitude);
        put("e", r.position.x);
        put("n", r.position.y);
        put("v", r.speed);
        put("vz", r.verticalVelocity);
        put("vh", r.horizontalSpeed);
        put("acc", r.acceleration.norm());
        put("axg", r.axialAcceleration / 9.80665);
        put("mach", r.mach);
        put("q", r.dynamicPressure / 1000.0);
        put("thrust", r.thrust);
        put("drag", r.dragForce);
        put("mass", r.mass);
        put("cd", r.cd);
        put("cdf", r.cdFriction);
        put("cdp", r.cdPressure);
        put("cdb", r.cdBase);
        put("stab", r.stabilityMargin);
        put("cg", r.cg);
        put("cp", r.cp);
        put("aoa", mathrix::rad2deg(r.angleOfAttack));
        put("zen", r.zenith);
        put("rate", mathrix::rad2deg(std::hypot(r.angularVelocity.y, r.angularVelocity.z)));
        put("recF", r.recoveryForce);
        put("wind", r.wind.norm());
        put("ph", static_cast<double>(r.phase));
    }
    return s;
}

json bodyJson(const BodyResult& b, int maxPoints) {
    const json s = seriesJson(b.records, maxPoints);
    json rec = json::array();
    for (const auto& x : b.recovery)
        rec.push_back({{"device", x.device},
                       {"type", x.type},
                       {"deployTime", r6(x.deployTime)},
                       {"deployAltitude", r6(x.deployAltitude)},
                       {"deploySpeed", r6(x.deploySpeed)},
                       {"peakForce", r6(x.peakForce)},
                       {"peakForceTime", r6(x.peakForceTime)},
                       {"dragArea", r6(x.dragArea)}});
    const auto& l = b.landing;
    return {{"name", b.name},
            {"parent", b.parent},
            {"apogee", r6(b.apogee())},
            {"mass", r6(b.mass)},
            {"recovery", rec},
            {"landing",
             {{"landed", l.landed},
              {"time", r6(l.time)},
              {"e", r6(l.position.x)},
              {"n", r6(l.position.y)},
              {"lat", l.latitude},
              {"lon", l.longitude},
              {"distance", r6(l.distance)},
              {"bearing", r6(l.bearing)},
              {"impactSpeed", r6(l.impactSpeed)},
              {"descentRate", r6(l.descentRate)},
              {"energy", r6(l.kineticEnergy)},
              {"drift", r6(l.driftFromApogee)}}},
            {"s", s}};
}

json buildData(const SimulationResult& result, const Rocket& rocket, const Environment& env, const ReportOptions& o) {
    json d;
    const MassProperties loaded = rocket.massProperties(-1.0), dry = rocket.dryMassProperties();
    AeroModel aero(rocket);
    json motors = json::array();
    for (const auto* m : rocket.motorMounts()) {
        if (!m->hasMotor()) continue;
        const Motor& mo = m->motor();
        std::vector<double> t, f;
        for (int i = 0; i <= 300; ++i) {
            const double tt = mo.burnTime() * 1.02 * i / 300.0;
            t.push_back(r6(tt));
            f.push_back(r6(mo.thrustAt(tt)));
        }
        motors.push_back({{"name", mo.displayName()},
                          {"class", mo.impulseClass()},
                          {"impulse", r6(mo.totalImpulse())},
                          {"burnTime", r6(mo.burnTime())},
                          {"avgThrust", r6(mo.averageThrust())},
                          {"maxThrust", r6(mo.maxThrust())},
                          {"isp", r6(mo.effectiveIsp())},
                          {"propellant", r6(mo.propellantMass)},
                          {"mount", m->name()},
                          {"source", mo.source.url},
                          {"interpolation", mathrix::toString(mo.interpolation)},
                          {"t", t},
                          {"f", f}});
    }
    // Design drag curve vs Mach at launch-site conditions.
    const AtmosphereState site = env.atmosphereAt(0.0);
    std::vector<double> M, cd, cdf, cdp, cdb, cpv;
    for (int i = 0; i <= 60; ++i) {
        FlightConditions c;
        c.mach = 0.02 + 2.48 * i / 60.0;
        c.airspeed = c.mach * site.speedOfSound;
        c.density = site.density;
        c.kinematicViscosity = site.kinematicViscosity();
        c.altitudeAsl = env.site().altitude;
        const AeroCoefficients a = aero.compute(c, loaded.cg);
        M.push_back(r6(c.mach));
        cd.push_back(r6(a.cd));
        cdf.push_back(r6(a.cdFriction));
        cdp.push_back(r6(a.cdPressure));
        cdb.push_back(r6(a.cdBase));
        cpv.push_back(r6(a.cp));
    }
    FlightConditions c03;
    c03.mach = 0.3;
    c03.airspeed = 0.3 * site.speedOfSound;
    c03.density = site.density;
    c03.kinematicViscosity = site.kinematicViscosity();
    const AeroCoefficients a03 = aero.compute(c03, loaded.cg);
    d["rocket"] = {{"name", rocket.name()},
                   {"designer", rocket.designer()},
                   {"description", rocket.description()},
                   {"length", r6(rocket.length())},
                   {"diameter", r6(rocket.referenceDiameter())},
                   {"massLoaded", r6(loaded.mass)},
                   {"massDry", r6(dry.mass)},
                   {"cgLoaded", r6(loaded.cg)},
                   {"cgDry", r6(dry.cg)},
                   {"cp", r6(a03.cp)},
                   {"cnAlpha", r6(a03.cnAlpha)},
                   {"dragSource", toString(rocket.aero().dragSource)},
                   {"motors", motors},
                   {"cdCurve", {{"mach", M}, {"cd", cd}, {"f", cdf}, {"p", cdp}, {"b", cdb}, {"cp", cpv}}}};
    // Atmosphere and wind profile up to above apogee.
    const double top = std::max(1000.0, result.summary.apogee * 1.15);
    std::vector<double> h, T, p, rho, ws;
    for (int i = 0; i <= 80; ++i) {
        const double z = top * i / 80.0;
        const AtmosphereState a = env.atmosphereAt(z);
        h.push_back(r6(z));
        T.push_back(r6(a.temperature - 273.15));
        p.push_back(r6(a.pressure / 1000.0));
        rho.push_back(r6(a.density));
        ws.push_back(r6(env.windAt(z, 0.0).norm()));
    }
    d["atmosphere"] = {{"h", h}, {"T", T}, {"p", p}, {"rho", rho}, {"wind", ws}};
    d["env"] = {{"planet", env.planet().name()},
                {"site", env.site().name},
                {"lat", env.site().latitude},
                {"lon", env.site().longitude},
                {"elevation", env.site().altitude},
                {"railLength", env.rail().length},
                {"railElevation", env.rail().elevation},
                {"railAzimuth", env.rail().azimuth},
                {"wind", env.wind().type()},
                {"gravity", r6(env.gravityAt(0.0))}};
    d["summary"] = result.summary.toJson();
    json ev = json::array();
    for (const auto& e : result.events)
        ev.push_back({{"t", r6(e.time)}, {"type", toString(e.type)}, {"body", e.body}, {"source", e.source},
                      {"alt", r6(e.altitude)}, {"v", r6(e.speed)}, {"msg", e.message}});
    d["events"] = ev;
    json bodies = json::array();
    for (const auto& b : result.bodies) bodies.push_back(bodyJson(b, o.maxPointsPerBody));
    d["bodies"] = bodies;
    // Launch vehicle history (ancestors + the body that reached the highest apogee).
    const size_t pi = result.primaryBody();
    if (pi < result.bodies.size()) {
        json pev = json::array();
        for (const auto& e : result.bodies[pi].events.all())
            pev.push_back({{"t", r6(e.time)}, {"type", toString(e.type)}, {"body", e.body}, {"source", e.source},
                           {"alt", r6(e.altitude)}, {"v", r6(e.speed)}});
        d["primary"] = {{"name", result.bodies[pi].name}, {"s", seriesJson(result.trajectory(pi), o.maxPointsPerBody)},
                        {"events", pev}};
    }
    d["warnings"] = result.warnings;
    d["compute"] = {{"steps", result.steps}, {"ms", r6(result.computeTime * 1000.0)}};
    return d;
}

std::string htmlEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '&') o += "&amp;";
        else o.push_back(c);
    }
    return o;
}

void replaceAll(std::string& s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
        s.replace(pos, from.size(), to);
        pos += to.size();
    }
}

// Split in two literals: MSVC limits a single string literal to ~16 KB.
const char* kTemplate = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{{TITLE}}</title>
<script src="{{PLOTLY}}"></script>
<style>
:root{--bg:#f6f7fb;--card:#ffffff;--ink:#1b1f3b;--muted:#5c6378;--line:#e3e6ef;--accent:#ff6b35;--accent2:#1c7ed6;--ok:#2f9e44;--warn:#e8590c;--grid:#e9ecf2}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#0d1124;--card:#151a33;--ink:#e9ecf5;--muted:#9aa3bd;--line:#252c4d;--accent:#ff8a5c;--accent2:#4dabf7;--ok:#51cf66;--warn:#ff922b;--grid:#232a48}}
:root[data-theme="dark"]{--bg:#0d1124;--card:#151a33;--ink:#e9ecf5;--muted:#9aa3bd;--line:#252c4d;--accent:#ff8a5c;--accent2:#4dabf7;--ok:#51cf66;--warn:#ff922b;--grid:#232a48}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;gap:18px;align-items:center;padding:22px 28px;background:linear-gradient(120deg,#141c48,#26387f);color:#fff}
header .logo{width:64px;height:64px;flex:none}header .logo svg{width:100%;height:100%}
header h1{margin:0;font-size:24px;letter-spacing:.3px}header p{margin:2px 0 0;opacity:.8;font-size:14px}
header .spacer{flex:1}header button{background:rgba(255,255,255,.12);color:#fff;border:1px solid rgba(255,255,255,.25);border-radius:8px;padding:6px 12px;cursor:pointer}
main{max-width:1320px;margin:0 auto;padding:22px 16px 60px}
.kpis{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:12px;margin-bottom:18px}
.kpi{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px}
.kpi .v{font-size:24px;font-weight:700;font-variant-numeric:tabular-nums}.kpi .l{color:var(--muted);font-size:12.5px;text-transform:uppercase;letter-spacing:.4px}
.kpi .s{color:var(--muted);font-size:12.5px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:16px;min-width:0}
.card h2{margin:0 0 10px;font-size:16px}.card h2 small{color:var(--muted);font-weight:400}
.drawing{color:var(--ink);overflow-x:auto}.drawing svg{width:100%;height:auto;max-height:260px}
.grid2{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}
@media (max-width:900px){.grid2{grid-template-columns:1fr}}
.chart{height:380px;width:100%}
table{width:100%;border-collapse:collapse;font-size:13.5px;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line);white-space:nowrap}th{color:var(--muted);font-weight:600}
.tbl{overflow-x:auto}.warn{color:var(--warn)}.muted{color:var(--muted)}.pill{display:inline-block;padding:1px 8px;border-radius:99px;background:var(--grid);font-size:12px}
footer{color:var(--muted);text-align:center;font-size:12.5px;margin-top:20px}
</style>
</head>
<body>
<header><div class="logo">{{LOGO}}</div><div><h1>{{TITLE}}</h1><p id="sub"></p></div><div class="spacer"></div>
<button id="theme" title="Toggle light/dark">&#9681; Theme</button></header>
<main>
<section class="kpis" id="kpis"></section>
<section class="card"><h2>Rocket <small id="rocketInfo"></small></h2><div class="drawing">{{DRAWING}}</div></section>
<div class="grid2">
<section class="card"><h2>Events</h2><div class="tbl"><table id="events"></table></div></section>
<section class="card"><h2>Bodies, recovery and landing</h2><div class="tbl"><table id="bodies"></table></div><div class="tbl" style="margin-top:10px"><table id="recovery"></table></div></section>
</div>
<div class="grid2">
<section class="card"><h2>Altitude</h2><div class="chart" id="c-alt"></div></section>
<section class="card"><h2>Velocity</h2><div class="chart" id="c-vel"></div></section>
<section class="card"><h2>Acceleration</h2><div class="chart" id="c-acc"></div></section>
<section class="card"><h2>Thrust, drag and mass</h2><div class="chart" id="c-forces"></div></section>
<section class="card"><h2>Mach number and dynamic pressure</h2><div class="chart" id="c-mach"></div></section>
<section class="card"><h2>Drag coefficient vs Mach <small>component build-up and flight values</small></h2><div class="chart" id="c-cd"></div></section>
<section class="card"><h2>Stability</h2><div class="chart" id="c-stab"></div></section>
<section class="card"><h2>Attitude</h2><div class="chart" id="c-aoa"></div></section>
<section class="card"><h2>3-D trajectory</h2><div class="chart" id="c-3d"></div></section>
<section class="card"><h2>Ground track and landing points</h2><div class="chart" id="c-ground"></div></section>
<section class="card"><h2>Descent rate</h2><div class="chart" id="c-descent"></div></section>
<section class="card"><h2>Recovery loads</h2><div class="chart" id="c-chute"></div></section>
<section class="card"><h2>Atmosphere</h2><div class="chart" id="c-atm"></div></section>
<section class="card"><h2>Wind profile</h2><div class="chart" id="c-wind"></div></section>
<section class="card"><h2>Motor thrust curve</h2><div class="chart" id="c-motor"></div></section>
<section class="card"><h2>Notes</h2><ul id="notes"></ul></section>
</div>
<footer>Generated by Rocket-Up {{VERSION}} on {{DATE}} &middot; <span id="compute"></span></footer>
</main>
)HTML"
                        R"HTML(<script>
const D = {{DATA}};
const COLORS=['#ff6b35','#1c7ed6','#2f9e44','#ae3ec9','#f59f00','#0ca678','#e64980'];
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt=(v,d=1)=>(v===undefined||v===null||isNaN(v))?'-':Number(v).toLocaleString('en-US',{maximumFractionDigits:d,minimumFractionDigits:d});
const main=D.primary||D.bodies[0];
function themeLayout(extra){const ink=css('--ink'),grid=css('--grid');return Object.assign({paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{color:ink,family:'system-ui,sans-serif',size:12},margin:{l:58,r:58,t:16,b:46},legend:{orientation:'h',y:-0.2},xaxis:{gridcolor:grid,zerolinecolor:grid},yaxis:{gridcolor:grid,zerolinecolor:grid},hovermode:'x unified'},extra||{});}
function ax(title,extra){return Object.assign({title:{text:title},gridcolor:css('--grid'),zerolinecolor:css('--grid')},extra||{});}
// Time axes of ascent charts open on powered flight + coast; double-click to see the whole flight.
function tax(){const t=D.summary.apogeeTime>0?D.summary.apogeeTime*1.15+2:undefined;return ax('Time (s)',t?{range:[0,t]}:{});}
const cfg={responsive:true,displaylogo:false,toImageButtonOptions:{format:'png',scale:2}};
function line(x,y,name,i,extra){return Object.assign({x,y,name,type:'scatter',mode:'lines',line:{width:2,color:COLORS[i%COLORS.length]}},extra||{});}
function eventShapes(){return (main.events||D.events).filter(e=>['burnout','apogee','deployment','rail-exit','ejection','separation'].includes(e.type)).map(e=>({type:'line',x0:e.t,x1:e.t,yref:'paper',y0:0,y1:1,line:{color:css('--muted'),width:1,dash:'dot'}}));}
function plot(id,traces,layout){Plotly.newPlot(id,traces,themeLayout(layout),cfg);}
function ascentMask(b){return b.s.ph.map(p=>p<=3);}
function filt(a,m){return a.filter((_,i)=>m[i]);}
function render(){
 const S=D.summary,R=D.rocket;
 document.getElementById('sub').textContent=`${D.env.planet} · ${D.env.site||'launch site'} (${fmt(D.env.lat,4)}°, ${fmt(D.env.lon,4)}°, ${fmt(D.env.elevation,0)} m) · rail ${fmt(D.env.railLength,1)} m @ ${fmt(D.env.railElevation,1)}°`;
 const k=[['Apogee',fmt(S.apogee,0)+' m',`ASL ${fmt(S.apogeeAsl,0)} m · t ${fmt(S.apogeeTime,1)} s`],['Max velocity',fmt(S.maxSpeed,1)+' m/s',`Mach ${fmt(S.maxMach,2)}`],['Max acceleration',fmt(S.maxAcceleration/9.80665,1)+' g',`${fmt(S.maxAcceleration,1)} m/s²`],['Rail exit',fmt(S.railExitSpeed,1)+' m/s',`t ${fmt(S.railExitTime,2)} s`],['Stability',fmt(S.stabilityAtRailExit,2)+' cal',`min ${fmt(S.minStability,2)} · max ${fmt(S.maxStability,2)}`],['Max q',fmt(S.maxDynamicPressure/1000,1)+' kPa',`max AoA ${fmt(S.maxAngleOfAttack,1)}°`],['Burnout',fmt(S.burnoutTime,2)+' s',`${fmt(S.burnoutAltitude,0)} m · ${fmt(S.burnoutSpeed,0)} m/s`],['Flight time',fmt(S.flightTime,1)+' s',`${D.bodies.length} bod${D.bodies.length>1?'ies':'y'}`]];
 document.getElementById('kpis').innerHTML=k.map(x=>`<div class="kpi"><div class="l">${x[0]}</div><div class="v">${x[1]}</div><div class="s">${x[2]}</div></div>`).join('');
 const mo=R.motors.map(m=>`${m.name} [${m.class}]`).join(', ');
 document.getElementById('rocketInfo').textContent=`· ${fmt(R.length,3)} m × ${fmt(R.diameter*1000,0)} mm · ${fmt(R.massLoaded,2)} kg loaded / ${fmt(R.massDry,2)} kg burnout · CNα ${fmt(R.cnAlpha,2)} · ${mo} · drag: ${R.dragSource}`;
 document.getElementById('events').innerHTML='<tr><th>t (s)</th><th>Event</th><th>Body</th><th>Source</th><th>Alt (m)</th><th>V (m/s)</th></tr>'+D.events.map(e=>`<tr><td>${fmt(e.t,2)}</td><td><span class="pill">${e.type}</span></td><td>${e.body}</td><td>${e.source||''}</td><td>${fmt(e.alt,1)}</td><td>${fmt(e.v,1)}</td></tr>`).join('');
 document.getElementById('bodies').innerHTML='<tr><th>Body</th><th>Apogee</th><th>Landing</th><th>Distance</th><th>Bearing</th><th>Impact</th><th>Descent</th><th>Energy</th><th>Drift</th></tr>'+D.bodies.map(b=>{const l=b.landing;return `<tr><td>${b.name}</td><td>${fmt(b.apogee,0)} m</td><td>${l.landed?fmt(l.time,1)+' s':'-'}</td><td>${fmt(l.distance,0)} m</td><td>${fmt(l.bearing,0)}°</td><td>${fmt(l.impactSpeed,1)} m/s</td><td>${fmt(l.descentRate,1)} m/s</td><td>${fmt(l.energy,0)} J</td><td>${fmt(l.drift,0)} m</td></tr>`}).join('');
 const rec=[];D.bodies.forEach(b=>b.recovery.forEach(r=>rec.push([b.name,r])));
 document.getElementById('recovery').innerHTML='<tr><th>Device</th><th>Body</th><th>Deploy t</th><th>Deploy alt</th><th>Deploy V</th><th>Peak load</th><th>Cd·A</th></tr>'+rec.map(([b,r])=>`<tr><td>${r.device}</td><td>${b}</td><td>${r.deployTime>=0?fmt(r.deployTime,2)+' s':'not deployed'}</td><td>${fmt(r.deployAltitude,0)} m</td><td>${fmt(r.deploySpeed,1)} m/s</td><td>${fmt(r.peakForce,0)} N</td><td>${fmt(r.dragArea,3)} m²</td></tr>`).join('');
 const notes=[...(S.notes||[]),...(D.warnings||[])];document.getElementById('notes').innerHTML=notes.length?notes.map(n=>`<li class="warn">${n}</li>`).join(''):'<li class="muted">No warnings.</li>';
 document.getElementById('compute').textContent=`${D.compute.steps} integration steps in ${fmt(D.compute.ms,1)} ms`;
 const ms=main.s,sh=eventShapes();
 const evPts=D.events.filter(e=>e.type!=='launch'&&e.type!=='ignition');
 plot('c-alt',[...D.bodies.map((b,i)=>line(b.s.t,b.s.alt,b.name,i)),{x:evPts.map(e=>e.t),y:evPts.map(e=>e.alt),text:evPts.map(e=>e.type+(e.source?': '+e.source:'')),mode:'markers',type:'scatter',name:'events',marker:{size:7,color:css('--ink'),symbol:'diamond'}}],{xaxis:ax('Time (s)'),yaxis:ax('Altitude AGL (m)'),hovermode:'closest'});
 plot('c-vel',[line(ms.t,ms.v,'total',0),line(ms.t,ms.vz,'vertical',1),line(ms.t,ms.vh,'horizontal',2)],{xaxis:tax(),yaxis:ax('Velocity (m/s)'),shapes:sh});
 plot('c-acc',[line(ms.t,ms.acc.map(a=>a/9.80665),'total (g)',0),line(ms.t,ms.axg,'axial / accelerometer (g)',1)],{xaxis:tax(),yaxis:ax('Acceleration (g)'),shapes:sh});
 plot('c-forces',[line(ms.t,ms.thrust,'thrust',0),line(ms.t,ms.drag,'drag',1),line(ms.t,ms.mass,'mass',2,{yaxis:'y2',line:{dash:'dash',width:2,color:COLORS[2]}})],{xaxis:tax(),yaxis:ax('Force (N)'),yaxis2:ax('Mass (kg)',{overlaying:'y',side:'right',showgrid:false}),shapes:sh});
 plot('c-mach',[line(ms.t,ms.mach,'Mach',0),line(ms.t,ms.q,'dynamic pressure',1,{yaxis:'y2'})],{xaxis:tax(),yaxis:ax('Mach'),yaxis2:ax('q (kPa)',{overlaying:'y',side:'right',showgrid:false}),shapes:sh});
 const am=ascentMask(main),c=R.cdCurve;
 plot('c-cd',[line(c.mach,c.cd,'total (design)',0),line(c.mach,c.f,'friction',1,{line:{dash:'dot',color:COLORS[1]}}),line(c.mach,c.p,'pressure',2,{line:{dash:'dot',color:COLORS[2]}}),line(c.mach,c.b,'base',3,{line:{dash:'dot',color:COLORS[3]}}),{x:filt(ms.mach,am),y:filt(ms.cd,am),name:'flight (ascent)',mode:'markers',type:'scatter',marker:{size:3,color:css('--ink'),opacity:.55}}],{xaxis:ax('Mach'),yaxis:ax('Cd'),hovermode:'closest'});
 plot('c-stab',[line(filt(ms.t,am),filt(ms.stab,am),'static margin (cal)',0),line(filt(ms.t,am),filt(ms.cg,am),'CG (m)',1,{yaxis:'y2'}),line(filt(ms.t,am),filt(ms.cp,am),'CP (m)',2,{yaxis:'y2'})],{xaxis:tax(),yaxis:ax('Stability (cal)'),yaxis2:ax('From nose tip (m)',{overlaying:'y',side:'right',showgrid:false}),shapes:sh});
 plot('c-aoa',[line(ms.t,ms.aoa,'angle of attack',0),line(ms.t,ms.zen,'zenith angle',1),line(ms.t,ms.rate,'pitch/yaw rate (deg/s)',2,{yaxis:'y2'})],{xaxis:tax(),yaxis:ax('Angle (deg)'),yaxis2:ax('Rate (deg/s)',{overlaying:'y',side:'right',showgrid:false}),shapes:sh});
 Plotly.newPlot('c-3d',D.bodies.map((b,i)=>({x:b.s.e,y:b.s.n,z:b.s.alt,name:b.name,type:'scatter3d',mode:'lines',line:{width:5,color:COLORS[i%COLORS.length]}})),themeLayout({margin:{l:0,r:0,t:0,b:0},scene:{xaxis:{title:'East (m)'},yaxis:{title:'North (m)'},zaxis:{title:'Altitude (m)'},aspectmode:'data'},hovermode:'closest'}),cfg);
 const land=D.bodies.filter(b=>b.landing.landed);
 plot('c-ground',[...D.bodies.map((b,i)=>line(b.s.e,b.s.n,b.name,i)),{x:[0],y:[0],name:'launch rail',mode:'markers',type:'scatter',marker:{size:12,symbol:'triangle-up',color:css('--ink')}},{x:land.map(b=>b.landing.e),y:land.map(b=>b.landing.n),text:land.map(b=>`${b.name}: ${fmt(b.landing.distance,0)} m @ ${fmt(b.landing.bearing,0)}°`),name:'landing',mode:'markers+text',textposition:'top center',type:'scatter',marker:{size:11,symbol:'x',color:css('--warn')}}],{xaxis:ax('East (m)'),yaxis:ax('North (m)',{scaleanchor:'x'}),hovermode:'closest'});
 plot('c-descent',D.bodies.map((b,i)=>{const m=b.s.ph.map((p,j)=>p>=4&&b.s.vz[j]<0);return line(filt(b.s.vz,m).map(v=>-v),filt(b.s.alt,m),b.name,i);}),{xaxis:ax('Descent rate (m/s)'),yaxis:ax('Altitude AGL (m)'),hovermode:'closest'});
 plot('c-chute',D.bodies.map((b,i)=>line(b.s.t,b.s.recF,b.name,i)),{xaxis:ax('Time (s)'),yaxis:ax('Recovery force (N)')});
 const A=D.atmosphere;
 plot('c-atm',[line(A.T,A.h,'temperature (°C)',0),line(A.p,A.h,'pressure (kPa)',1,{xaxis:'x2'}),line(A.rho.map(r=>r*100),A.h,'density (×0.01 kg/m³)',2,{xaxis:'x2'})],{xaxis:ax('Temperature (°C)'),xaxis2:ax('Pressure (kPa) / density',{overlaying:'x',side:'top',showgrid:false}),yaxis:ax('Altitude AGL (m)'),hovermode:'closest',margin:{l:58,r:30,t:46,b:46}});
 plot('c-wind',[line(A.wind,A.h,'wind speed',0),line(ms.wind,ms.alt,'wind seen by the rocket',1,{mode:'markers',marker:{size:3}})],{xaxis:ax('Wind speed (m/s)'),yaxis:ax('Altitude AGL (m)'),hovermode:'closest'});
 plot('c-motor',R.motors.map((m,i)=>line(m.t,m.f,`${m.name}: ${fmt(m.impulse,0)} Ns, ${fmt(m.burnTime,2)} s, Isp ${fmt(m.isp,1)} s, ${m.interpolation} interpolation`,i,{fill:'tozeroy'})),{xaxis:ax('Time (s)'),yaxis:ax('Thrust (N)')});
}
render();
document.getElementById('theme').onclick=()=>{const r=document.documentElement;const dark=r.dataset.theme?r.dataset.theme==='dark':matchMedia('(prefers-color-scheme: dark)').matches;r.dataset.theme=dark?'light':'dark';render();};
</script>
</body>
</html>
)HTML";

}  // namespace

std::string htmlReport(const SimulationResult& result, const Rocket& rocket, const Environment& env,
                       const ReportOptions& o) {
    std::string html = kTemplate;
    const std::string title = o.title.empty() ? rocket.name() + " flight report" : o.title;
    DrawingOptions dopt;
    dopt.standalone = false;
    std::string data = buildData(result, rocket, env, o).dump();
    replaceAll(data, "</", "<\\/");  // keep the JSON safe inside <script>
    char date[64];
    const std::time_t now = std::time(nullptr);
    std::strftime(date, sizeof date, "%Y-%m-%d %H:%M", std::localtime(&now));
    replaceAll(html, "{{TITLE}}", htmlEscape(title));
    replaceAll(html, "{{PLOTLY}}", o.plotlyUrl);
    replaceAll(html, "{{LOGO}}", logoSvg());
    replaceAll(html, "{{DRAWING}}", drawRocketSvg(rocket, dopt));
    replaceAll(html, "{{VERSION}}", "0.1.0");
    replaceAll(html, "{{DATE}}", date);
    replaceAll(html, "{{DATA}}", data);
    return html;
}

void writeHtmlReport(const SimulationResult& result, const Rocket& rocket, const Environment& env,
                     const std::string& path, const ReportOptions& options) {
    writeTextFile(path, htmlReport(result, rocket, env, options));
}

}  // namespace rocketup::io
