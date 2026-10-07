#include "rocketup/sim/Simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <nlohmann/json.hpp>

#include "rocketup/aero/AeroModel.hpp"

namespace rocketup {

using mathrix::kPi;

const char* toString(DynamicsModel d) { return d == DynamicsModel::SixDof ? "6dof" : "3dof"; }

DynamicsModel dynamicsModelFromString(const std::string& s) {
    if (s == "6dof" || s == "six-dof" || s == "6-dof") return DynamicsModel::SixDof;
    if (s == "3dof" || s == "three-dof" || s == "3-dof" || s == "point-mass") return DynamicsModel::ThreeDof;
    throw RocketUpError("unknown dynamics model '" + s + "' (use 6dof or 3dof)");
}

json SimulationOptions::toJson() const {
    return {{"integrator", mathrix::toString(integrator)},
            {"dynamics", toString(dynamics)},
            {"timeStep", timeStep},
            {"minTimeStep", minTimeStep},
            {"maxTimeStep", maxTimeStep},
            {"absoluteTolerance", absoluteTolerance},
            {"relativeTolerance", relativeTolerance},
            {"maxTime", maxTime},
            {"stopAtApogee", stopAtApogee},
            {"recordInterval", recordInterval},
            {"descentRecordInterval", descentRecordInterval},
            {"sensors", sensors.toJson()},
            {"seed", seed}};
}

SimulationOptions SimulationOptions::fromJson(const json& j) {
    SimulationOptions o;
    if (j.contains("integrator")) o.integrator = mathrix::integratorFromString(j.at("integrator").get<std::string>());
    if (j.contains("dynamics")) o.dynamics = dynamicsModelFromString(j.at("dynamics").get<std::string>());
    o.timeStep = j.value("timeStep", o.timeStep);
    o.minTimeStep = j.value("minTimeStep", o.minTimeStep);
    o.maxTimeStep = j.value("maxTimeStep", o.maxTimeStep);
    o.absoluteTolerance = j.value("absoluteTolerance", o.absoluteTolerance);
    o.relativeTolerance = j.value("relativeTolerance", o.relativeTolerance);
    o.maxTime = j.value("maxTime", o.maxTime);
    o.stopAtApogee = j.value("stopAtApogee", o.stopAtApogee);
    o.recordInterval = j.value("recordInterval", o.recordInterval);
    o.descentRecordInterval = j.value("descentRecordInterval", o.descentRecordInterval);
    if (j.contains("sensors")) o.sensors = SensorConfig::fromJson(j.at("sensors"));
    o.seed = j.value("seed", o.seed);
    if (o.timeStep <= 0.0) throw RocketUpError("time step must be positive");
    return o;
}

namespace {

using StateVec = mathrix::VecN<13>;

enum class Mode { Constrained, Free6, Free3, Descent };

/// Remembers when a trigger first became true and fires after its latch delay.
struct Latch {
    double firstTrue = -1.0;
    bool fired = false;
    bool update(const Trigger& t, const TriggerContext& c) {
        if (fired || t.isNever()) return false;
        if (firstTrue < 0.0 && t(c)) firstTrue = c.time();
        if (firstTrue >= 0.0 && c.time() + 1e-9 >= firstTrue + t.delay()) {
            fired = true;
            return true;
        }
        return false;
    }
};

struct MotorState {
    MotorState(const MotorMount* m, double px) : mount(m), x(px) {}
    const MotorMount* mount;
    double x;
    double ignition = -1.0;
    bool burnout = false;
    Latch latch;
};
struct RecoveryState {
    RecoveryState(const RecoveryDevice* d, double px) : device(d), x(px) {}
    const RecoveryDevice* device;
    double x;
    Latch latch;
    double deployStart = -1.0;
    bool deployed = false;
    RecoveryReport report;
};
struct PayloadState {
    PayloadState(const Payload* p, double px) : payload(p), x(px) {}
    const Payload* payload;
    double x;
    Latch latch;
};
struct SeparationState {
    SeparationState(const Separation* s, int idx) : sep(s), stackIndex(idx) {}
    const Separation* sep;
    int stackIndex;
    Latch latch;
};

struct Diag {
    AeroCoefficients aero;
    AtmosphereState atm;
    double thrust = 0, g = 0, mass = 0, cg = 0, ixx = 0, iyy = 0, prop = 0;
    double airspeed = 0, mach = 0, q = 0, aoa = 0;
    double dragForce = 0, normalForce = 0;
    double recoveryCdA = 0, recoveryForce = 0;
    Vec3 accel, specificForceBody, wind, axis;
};

struct Body {
    std::string name, parent;
    double startTime = 0.0;
    std::vector<PlacedComponent> parts;
    MassProperties structure;
    std::unique_ptr<AeroModel> aero;
    double payloadCdA = 0.0;
    std::vector<MotorState> motors;
    std::vector<RecoveryState> recovery;
    std::vector<PayloadState> payloads;
    std::vector<SeparationState> separations;
    Mode mode = Mode::Constrained;
    double t = 0.0, h = 0.01;
    StateVec y;
    bool liftoff = false, railExit = false, apogee = false, landed = false;
    double maxAltitude = 0.0;
    Vec3 apogeePosition;
    bool hasApogeePosition = false;
    EventLog events;
    std::unique_ptr<SensorSimulator> sensors;
    FlightSnapshot snap;
    BodyResult result;
    double lastRecord = -1e300;
    int index = 0;
};

Vec3 posOf(const StateVec& y) { return {y[0], y[1], y[2]}; }
Vec3 velOf(const StateVec& y) { return {y[3], y[4], y[5]}; }
Quat quatOf(const StateVec& y) { return Quat{y[6], y[7], y[8], y[9]}.normalized(); }
Vec3 omegaOf(const StateVec& y) { return {y[10], y[11], y[12]}; }
void setVec(StateVec& y, size_t i, const Vec3& v) {
    y[i] = v.x;
    y[i + 1] = v.y;
    y[i + 2] = v.z;
}
void setQuat(StateVec& y, const Quat& q) {
    y[6] = q.w;
    y[7] = q.x;
    y[8] = q.y;
    y[9] = q.z;
}

bool isDescendantOf(const Component* c, const Component* ancestor) {
    for (const Component* p = c; p; p = p->parent())
        if (p == ancestor) return true;
    return false;
}

class Engine {
public:
    Engine(const Rocket& rocket, const Environment& env, const SimulationOptions& opt,
           const std::vector<std::shared_ptr<SimulationObserver>>& observers,
           const std::function<bool(double, double)>& progress, std::atomic<bool>& cancel)
        : rocket_(rocket), env_(env), opt_(opt), observers_(observers), progress_(progress), cancel_(cancel) {
        railDir_ = env_.rail().direction();
        railAttitude_ = Quat::fromTwoVectors(Vec3::unitX(), railDir_);
        // A measured dry mass is distributed over the computed structural parts only: parts
        // with an explicitly known mass (mass objects, payloads, overrides) keep it.
        double explicitMass = 0.0, computedMass = 0.0;
        for (const auto& p : rocket_.flatten()) {
            const double m = p.component->mass().mass;
            (isExplicitMass(*p.component) ? explicitMass : computedMass) += m;
        }
        if (rocket_.dryMassOverride()) {
            const double target = rocket_.dryMassOverride()->mass;
            if (computedMass > 1e-9 && target > explicitMass) {
                massScale_ = (target - explicitMass) / computedMass;
            } else if (explicitMass + computedMass > 0.0) {
                massScale_ = target / (explicitMass + computedMass);
                explicitScale_ = massScale_;
            }
        }
    }

    SimulationResult run(const Simulation& sim) {
        const auto wall0 = std::chrono::steady_clock::now();
        SimulationResult result;
        result.rocketName = rocket_.name();
        result.warnings = rocket_.validate();
        for (auto& o : observers_) o->onStart(sim);

        StateVec y0;
        setQuat(y0, railAttitude_);
        auto first = makeBody(rocket_.name(), "", rocket_.flatten(), 0.0, y0, Mode::Constrained, EventLog{}, true);
        pending_.push_back(std::move(first));

        while (!pending_.empty()) {
            std::unique_ptr<Body> b = std::move(pending_.front());
            pending_.pop_front();
            const bool primary = finished_.empty() && b->parent.empty();
            fly(*b, primary);
            finishBody(*b);
            finished_.push_back(std::move(b->result));
            if (cancel_) break;
        }
        for (auto& b : finished_) {
            for (const auto& e : b.events.all())
                if (e.body == b.name) allEvents_.push_back(e);
        }
        std::stable_sort(allEvents_.begin(), allEvents_.end(),
                         [](const FlightEvent& a, const FlightEvent& b) { return a.time < b.time; });
        result.bodies = std::move(finished_);
        result.events = std::move(allEvents_);
        for (auto& w : warnings_) result.warnings.push_back(w);
        if (cancel_) result.warnings.push_back("simulation cancelled");
        result.summary = computeSummary(result);
        result.steps = steps_;
        result.computeTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
        for (auto& o : observers_) o->onFinish(result);
        return result;
    }

private:
    const Rocket& rocket_;
    const Environment& env_;
    const SimulationOptions& opt_;
    const std::vector<std::shared_ptr<SimulationObserver>>& observers_;
    const std::function<bool(double, double)>& progress_;
    std::atomic<bool>& cancel_;
    Vec3 railDir_;
    Quat railAttitude_;
    double massScale_ = 1.0;      ///< applied to computed structural masses
    double explicitScale_ = 1.0;  ///< applied to explicitly specified masses (normally 1)
    ExternalCommands commands_;

    static bool isExplicitMass(const Component& c) {
        return c.massOverride().has_value() || dynamic_cast<const MassObject*>(&c) != nullptr;
    }
    std::deque<std::unique_ptr<Body>> pending_;
    std::vector<BodyResult> finished_;
    std::vector<FlightEvent> allEvents_;
    std::vector<std::string> warnings_;
    long long steps_ = 0;
    int bodyCounter_ = 0;

    // ------------------------------------------------------------ body construction

    std::unique_ptr<Body> makeBody(const std::string& name, const std::string& parent, std::vector<PlacedComponent> parts,
                                   double t, const StateVec& y, Mode mode, EventLog events, bool intact) {
        auto b = std::make_unique<Body>();
        b->name = name;
        b->parent = parent;
        b->startTime = t;
        b->t = t;
        b->h = opt_.timeStep;
        b->y = y;
        b->mode = mode;
        b->events = std::move(events);
        b->parts = std::move(parts);
        b->index = bodyCounter_++;
        b->sensors = std::make_unique<SensorSimulator>(opt_.sensors, opt_.seed + 7919ULL * static_cast<uint64_t>(b->index));
        rebuild(*b, intact);
        b->result.name = name;
        b->result.parent = parent;
        b->result.startTime = t;
        return b;
    }

    void rebuild(Body& b, bool intact) {
        b.structure = MassProperties{};
        if (intact && rocket_.dryMassOverride()) {
            b.structure = *rocket_.dryMassOverride();
        } else {
            for (const auto& p : b.parts) {
                const MassProperties m = p.component->mass();
                const double k = isExplicitMass(*p.component) ? explicitScale_ : massScale_;
                b.structure += m.scaledTo(m.mass * k).shifted(p.position);
            }
        }
        b.aero = std::make_unique<AeroModel>(b.parts, rocket_.aero(), rocket_.referenceDiameter());
        b.payloadCdA = 0.0;
        // Keep runtime states of components that are still part of the body.
        auto keepMotors = std::move(b.motors);
        auto keepRecovery = std::move(b.recovery);
        auto keepPayloads = std::move(b.payloads);
        auto keepSeps = std::move(b.separations);
        b.motors.clear();
        b.recovery.clear();
        b.payloads.clear();
        b.separations.clear();
        int minStack = 1 << 30, maxStack = -1;
        for (const auto& p : b.parts) {
            const Component* c = p.component;
            if (c->isBody()) {
                minStack = std::min(minStack, p.stackIndex);
                maxStack = std::max(maxStack, p.stackIndex);
            }
            if (auto* mm = dynamic_cast<const MotorMount*>(c); mm && mm->hasMotor()) {
                MotorState s{mm, p.position};
                for (auto& k : keepMotors)
                    if (k.mount == mm) s = k;
                b.motors.push_back(s);
            } else if (auto* rd = dynamic_cast<const RecoveryDevice*>(c)) {
                RecoveryState s{rd, p.position};
                s.report.device = rd->name();
                s.report.type = rd->type();
                s.report.dragArea = rd->dragArea();
                for (auto& k : keepRecovery)
                    if (k.device == rd) s = k;
                b.recovery.push_back(s);
            } else if (auto* pl = dynamic_cast<const Payload*>(c)) {
                // A payload that already is its own body cannot be ejected again.
                if (!pl->ejectTrigger().isNever() && b.parts.front().component != pl) {
                    PayloadState s{pl, p.position};
                    for (auto& k : keepPayloads)
                        if (k.payload == pl) s = k;
                    b.payloads.push_back(s);
                }
            }
        }
        for (const auto& sep : rocket_.separations()) {
            const int idx = rocket_.stackIndexOf(sep.aftSectionStart);
            if (idx > minStack && idx <= maxStack) {
                SeparationState s{&sep, idx};
                for (auto& k : keepSeps)
                    if (k.sep == &sep) s = k;
                b.separations.push_back(s);
            }
        }
        if (!b.aero->hasBody()) {
            for (const auto& p : b.parts)
                if (auto* pl = dynamic_cast<const Payload*>(p.component)) b.payloadCdA += pl->freeFallDragArea();
            if (b.payloadCdA <= 0.0) b.payloadCdA = 0.01;
        }
    }

    // ------------------------------------------------------------ dynamics

    StateVec deriv(const Body& b, double t, const StateVec& y, Diag* d) const {
        const Vec3 pos = posOf(y), vel = velOf(y), w = omegaOf(y);
        const Quat q = b.mode == Mode::Constrained ? railAttitude_ : quatOf(y);
        const double alt = pos.z;
        const AtmosphereState atm = env_.atmosphereAt(alt);
        const double g = env_.gravityAt(alt);
        const Vec3 wind = env_.windAt(std::max(alt, 0.0), t);

        MassProperties mp = b.structure;
        double thrust = 0.0, prop = 0.0, plume = 0.0;
        for (const auto& m : b.motors) {
            const double tm = m.ignition < 0.0 ? -1.0 : t - m.ignition;
            mp += m.mount->motorMass(tm).shifted(m.x);
            const Motor& motor = m.mount->motor();
            prop += tm < 0.0 ? motor.propellantMass : motor.propellantMassAt(tm);
            if (tm >= 0.0) {
                const double f = motor.thrustAt(tm, atm.pressure);
                thrust += f;
                if (f > 0.0) plume += kPi * 0.25 * motor.diameter * motor.diameter;
            }
        }
        const double mass = std::max(mp.mass, 1e-6);
        const Vec3 vAir = vel - wind;
        const double V = vAir.norm();

        FlightConditions fc;
        fc.airspeed = V;
        fc.mach = V / atm.speedOfSound;
        fc.density = atm.density;
        fc.kinematicViscosity = atm.kinematicViscosity();
        fc.altitudeAsl = env_.site().altitude + alt;
        fc.plumeArea = rocket_.aero().plumeReducesBaseDrag ? plume : 0.0;
        fc.rollRate = w.x;
        const double qd = fc.dynamicPressure();

        Vec3 force{0.0, 0.0, -mass * g};
        Vec3 moment;
        Vec3 axis = q.rotate(Vec3::unitX());
        AeroCoefficients ac;
        double aoa = 0.0, dragForce = 0.0, normalForce = 0.0, recCdA = 0.0;
        StateVec dy;
        setVec(dy, 0, vel);

        auto aeroBodyForces = [&](Vec3& fBody, Vec3& mBody) {
            const Vec3 u = q.inverseRotate(vAir);
            aoa = V > 1e-9 ? std::acos(mathrix::clamp(u.x / V, -1.0, 1.0)) : 0.0;
            fc.angleOfAttack = aoa;
            ac = b.aero->compute(fc, mp.cg);
            const double A = ac.referenceArea, dref = ac.referenceLength;
            const Vec3 lat{0.0, u.y, u.z};
            const double latN = lat.norm();
            const Vec3 nDir = latN > 1e-12 ? lat * (-1.0 / latN) : Vec3{};
            fBody = Vec3{-qd * A * ac.axial, 0.0, 0.0} + nDir * (qd * A * ac.normal);
            const Vec3 rcp{mp.cg - ac.cp, 0.0, 0.0};
            mBody = rcp.cross(nDir * (qd * A * ac.normal));
            const Vec3 wPerp{0.0, w.y, w.z};
            mBody -= wPerp * (wPerp.norm() * 0.5 * atm.density * A * dref * ac.pitchDamping);
            mBody.x += qd * A * dref * (ac.rollForcing - ac.rollDamping);
            dragForce = qd * A * ac.totalDrag(aoa);
            normalForce = qd * A * ac.normal;
        };

        switch (b.mode) {
            case Mode::Constrained: {
                Vec3 fb, mb;
                aeroBodyForces(fb, mb);
                force += q.rotate(fb) + axis * thrust;
                double along = force.dot(axis) / mass;
                const double fr = env_.rail().frictionCoefficient;
                if (fr > 0.0) along -= fr * (force - axis * force.dot(axis)).norm() / mass;
                if (vel.dot(axis) <= 1e-9 && along < 0.0) along = 0.0;
                setVec(dy, 3, axis * along);
                force = axis * (along * mass);
                break;
            }
            case Mode::Free6: {
                Vec3 fb, mb;
                aeroBodyForces(fb, mb);
                force += q.rotate(fb) + axis * thrust;
                Vec3 acc = force / mass;
                if (env_.rotatingFrame()) acc -= env_.planetRotationEnu().cross(vel) * 2.0;
                setVec(dy, 3, acc);
                const double ixx = std::max(mp.ixx, 1e-9), iyy = std::max(mp.iyy, 1e-9);
                const Vec3 Iw{ixx * w.x, iyy * w.y, iyy * w.z};
                const Vec3 rhs = mb - w.cross(Iw);
                setVec(dy, 10, Vec3{rhs.x / ixx, rhs.y / iyy, rhs.z / iyy});
                const Quat qd4 = q.derivative(w);
                dy[6] = qd4.w;
                dy[7] = qd4.x;
                dy[8] = qd4.y;
                dy[9] = qd4.z;
                break;
            }
            case Mode::Free3: {
                if (V > 1e-6) axis = vAir / V;
                fc.angleOfAttack = 0.0;
                ac = b.aero->compute(fc, mp.cg);
                dragForce = qd * ac.referenceArea * ac.axial;
                force += axis * (thrust - dragForce);
                Vec3 acc = force / mass;
                if (env_.rotatingFrame()) acc -= env_.planetRotationEnu().cross(vel) * 2.0;
                setVec(dy, 3, acc);
                break;
            }
            case Mode::Descent: {
                if (V > 1e-6) axis = vAir / V;
                for (const auto& r : b.recovery)
                    if (r.deployStart >= 0.0 && t >= r.deployStart) recCdA += r.device->dragAreaAt(t - r.deployStart);
                const double bodyCdA = b.aero->hasBody() ? b.aero->tumblingDragArea() : b.payloadCdA;
                fc.angleOfAttack = 0.0;
                if (b.aero->hasBody()) ac = b.aero->compute(fc, mp.cg);
                ac.referenceArea = b.aero->referenceArea();
                ac.cd = (recCdA + bodyCdA) / ac.referenceArea;
                dragForce = qd * (recCdA + bodyCdA);
                force += axis * (thrust - dragForce);
                Vec3 acc = force / mass;
                if (env_.rotatingFrame()) acc -= env_.planetRotationEnu().cross(vel) * 2.0;
                setVec(dy, 3, acc);
                break;
            }
        }

        if (d) {
            d->aero = ac;
            d->atm = atm;
            d->thrust = thrust;
            d->g = g;
            d->mass = mass;
            d->cg = mp.cg;
            d->ixx = mp.ixx;
            d->iyy = mp.iyy;
            d->prop = prop;
            d->airspeed = V;
            d->mach = fc.mach;
            d->q = qd;
            d->aoa = aoa;
            d->dragForce = dragForce;
            d->normalForce = normalForce;
            d->recoveryCdA = recCdA;
            d->recoveryForce = qd * recCdA;
            d->accel = velOf(dy);
            d->specificForceBody = (b.mode == Mode::Constrained || b.mode == Mode::Free6 ? q : Quat::fromTwoVectors(Vec3::unitX(), axis))
                                       .inverseRotate(d->accel + Vec3{0.0, 0.0, g});
            d->wind = wind;
            d->axis = axis;
        }
        return dy;
    }

    // ------------------------------------------------------------ main loop

    void fly(Body& b, bool primary) {
        // Initial evaluation (ignite motors with launch triggers, record t0).
        if (b.events.all().empty()) emit(b, EventType::Launch, "", "launch");
        Diag d;
        deriv(b, b.t, b.y, &d);
        updateSnapshot(b, d);
        processTriggers(b);
        deriv(b, b.t, b.y, &d);
        updateSnapshot(b, d);
        record(b, d, true);

        auto f = [&](double t, const StateVec& y) { return deriv(b, t, y, nullptr); };
        long long localSteps = 0;
        bool firstStepLogged = false;
        while (!b.landed && !cancel_) {
            if (b.t >= opt_.maxTime) {
                warnings_.push_back("body '" + b.name + "' reached the maximum simulation time");
                break;
            }
            const StateVec prev = b.y;
            const double tPrev = b.t;
            double h = b.h;
            StateVec yNew;
            if (mathrix::isAdaptive(opt_.integrator)) {
                for (;;) {
                    const auto trial = mathrix::dormandPrinceStep(f, b.t, b.y, h, opt_.absoluteTolerance, opt_.relativeTolerance);
                    if (trial.error <= 1.0 || h <= opt_.minTimeStep * 1.0001) {
                        yNew = trial.y;
                        b.h = std::clamp(mathrix::nextStepSize(h, trial.error), opt_.minTimeStep, opt_.maxTimeStep);
                        break;
                    }
                    h = std::max(opt_.minTimeStep, mathrix::nextStepSize(h, trial.error));
                }
            } else {
                yNew = mathrix::fixedStep(opt_.integrator, f, b.t, b.y, h);
            }
            setQuat(yNew, b.mode == Mode::Free6 ? quatOf(yNew) : quatOf(prev));
            bool finite = true;
            for (size_t i = 0; i < StateVec::size(); ++i) finite = finite && std::isfinite(yNew[i]);
            if (!finite) {
                warnings_.push_back("body '" + b.name + "': numerical instability at t = " + std::to_string(b.t) +
                                    " s (reduce the time step or use rk4/rk45)");
                break;
            }
            b.y = yNew;
            b.t = tPrev + h;
            ++steps_;
            ++localSteps;
            bool forceRecord = false;

            // ---- launch rail
            if (b.mode == Mode::Constrained) {
                const double vAlong = velOf(b.y).dot(railDir_);
                if (!b.liftoff && vAlong > 1e-3) {
                    b.liftoff = true;
                    emit(b, EventType::Liftoff, "", "liftoff");
                    forceRecord = true;
                }
                if (posOf(b.y).dot(railDir_) >= env_.rail().length) {
                    b.railExit = true;
                    b.mode = opt_.dynamics == DynamicsModel::SixDof ? Mode::Free6 : Mode::Free3;
                    setQuat(b.y, railAttitude_);
                    setVec(b.y, 10, Vec3{});
                    emit(b, EventType::RailExit, "", "rail exit");
                    forceRecord = true;
                }
                if (!b.liftoff && allMotorsDone(b) && b.t > 1.0) {
                    warnings_.push_back("rocket did not lift off (thrust never exceeded weight)");
                    break;
                }
            }

            // ---- ground contact
            if (b.liftoff && b.y[2] <= 0.0 && b.t - b.startTime > 0.05 && b.y[5] < 0.0) {
                const double z0 = prev[2], z1 = b.y[2];
                const double frac = z0 > z1 ? mathrix::clamp(z0 / (z0 - z1), 0.0, 1.0) : 1.0;
                b.y = prev + (b.y - prev) * frac;
                b.y[2] = 0.0;
                b.t = tPrev + frac * h;
                b.landed = true;
            }

            Diag d2;
            deriv(b, b.t, b.y, &d2);

            // ---- apogee
            if ((b.liftoff || !b.parent.empty()) && !b.apogee && prev[5] > 0.0 && b.y[5] <= 0.0) {
                const double fr = prev[5] / (prev[5] - b.y[5]);
                const double tA = tPrev + fr * (b.t - tPrev);
                const double zA = prev[2] + 0.5 * prev[5] * (tA - tPrev);
                b.apogee = true;
                b.apogeePosition = posOf(prev) + (posOf(b.y) - posOf(prev)) * fr;
                b.hasApogeePosition = true;
                FlightEvent e{tA, EventType::Apogee, b.name, "", "apogee", zA, (velOf(prev) * (1 - fr) + velOf(b.y) * fr).norm()};
                pushEvent(b, e);
                forceRecord = true;
            }
            b.maxAltitude = std::max(b.maxAltitude, b.y[2]);

            // ---- burnout
            for (auto& m : b.motors) {
                if (m.ignition >= 0.0 && !m.burnout && b.t - m.ignition >= m.mount->motor().burnTime()) {
                    m.burnout = true;
                    emit(b, EventType::Burnout, m.mount->name(), "burnout of " + m.mount->motor().displayName());
                    forceRecord = true;
                }
            }

            updateSnapshot(b, d2);
            if (!b.landed) forceRecord |= processTriggers(b);
            deriv(b, b.t, b.y, &d2);
            updateSnapshot(b, d2);
            trackRecovery(b, d2);
            if (b.landed) {
                emit(b, EventType::Landing, "", "landing");
                forceRecord = true;
            }
            record(b, d2, forceRecord || !firstStepLogged);
            firstStepLogged = true;

            if (primary && opt_.stopAtApogee && b.apogee) break;
            if (progress_ && localSteps % 200 == 0 && !progress_(b.t, b.y[2])) cancel_ = true;
        }
    }

    bool allMotorsDone(const Body& b) const {
        for (const auto& m : b.motors)
            if (!m.burnout) return false;
        return true;
    }

    void updateSnapshot(Body& b, const Diag& d) {
        FlightSnapshot& s = b.snap;
        s.time = b.t;
        s.position = posOf(b.y);
        s.velocity = velOf(b.y);
        s.acceleration = d.accel;
        s.altitude = s.position.z;
        s.altitudeAsl = env_.site().altitude + s.position.z;
        s.verticalVelocity = s.velocity.z;
        s.speed = s.velocity.norm();
        s.airspeed = d.airspeed;
        s.mach = d.mach;
        s.dynamicPressure = d.q;
        s.angleOfAttack = d.aoa;
        s.mass = d.mass;
        s.thrust = d.thrust;
        s.attitude = b.mode == Mode::Free6 ? quatOf(b.y) : (b.mode == Mode::Constrained ? railAttitude_ : Quat::fromTwoVectors(Vec3::unitX(), d.axis));
        s.angularVelocity = b.mode == Mode::Free6 ? omegaOf(b.y) : Vec3{};
        s.maxAltitude = b.maxAltitude;
        if (b.landed) s.phase = FlightPhase::Landed;
        else if (b.mode == Mode::Constrained) s.phase = b.liftoff ? FlightPhase::OnRail : FlightPhase::OnPad;
        else if (b.mode == Mode::Descent) s.phase = FlightPhase::Descent;
        else s.phase = d.thrust > 0.0 ? FlightPhase::Powered : FlightPhase::Coasting;
        double lat, lon;
        env_.toGeodetic(s.position, lat, lon);
        b.sensors->update(b.t, s, d.specificForceBody, d.atm.pressure, d.atm.temperature, lat, lon);
    }

    void trackRecovery(Body& b, const Diag& d) {
        for (auto& r : b.recovery) {
            if (r.deployStart < 0.0 || b.t < r.deployStart) continue;
            const double force = d.q * r.device->dragAreaAt(b.t - r.deployStart);
            if (force > r.report.peakForce) {
                r.report.peakForce = force;
                r.report.peakForceTime = b.t;
            }
        }
    }

    /// Evaluates every trigger of the body. Returns true if something happened.
    bool processTriggers(Body& b) {
        for (auto& o : observers_) o->poll(b.t, commands_);
        const TriggerContext ctx{b.snap, b.sensors->readings(), b.events, commands_, b.name};
        bool any = false;
        for (auto& m : b.motors) {
            if (m.ignition < 0.0 && m.latch.update(m.mount->ignitionTrigger(), ctx)) {
                m.ignition = b.t;
                emit(b, EventType::Ignition, m.mount->name(), "ignition of " + m.mount->motor().displayName());
                any = true;
            }
        }
        for (auto& r : b.recovery) {
            if (r.deployStart < 0.0 && r.latch.update(r.device->deployTrigger(), ctx)) {
                r.report.triggerTime = b.t;
                r.deployStart = b.t + r.device->deployDelay();
            }
            if (r.deployStart >= 0.0 && !r.deployed && b.t + 1e-9 >= r.deployStart) {
                r.deployed = true;
                r.report.deployTime = b.t;
                r.report.deployAltitude = b.snap.altitude;
                r.report.deploySpeed = b.snap.speed;
                emit(b, EventType::Deployment, r.device->name(), "deployment of " + r.device->name());
                enterDescent(b);
                any = true;
            }
        }
        // Payload ejections and separations change the body: collect first.
        std::vector<const Payload*> ejections;
        for (auto& p : b.payloads)
            if (p.latch.update(p.payload->ejectTrigger(), ctx)) ejections.push_back(p.payload);
        std::vector<const Separation*> seps;
        for (auto& s : b.separations)
            if (s.latch.update(s.sep->trigger, ctx)) seps.push_back(s.sep);
        for (const Payload* p : ejections) {
            eject(b, p);
            any = true;
        }
        for (const Separation* s : seps) {
            separate(b, *s);
            any = true;
        }
        return any;
    }

    void enterDescent(Body& b) {
        if (b.mode == Mode::Descent || b.mode == Mode::Constrained) return;
        b.mode = Mode::Descent;
        setVec(b.y, 10, Vec3{});
    }

    Mode modeForSplitBody(const Body& b) const {
        bool deployed = false;
        for (const auto& r : b.recovery) deployed |= r.deployed;
        // Sections with a nose or with fins fly aerodynamically; others tumble.
        if (!deployed && (b.aero->hasNose() || b.aero->hasFins()) && b.y[5] > 0.0 && !b.apogee)
            return opt_.dynamics == DynamicsModel::SixDof ? Mode::Free6 : Mode::Free3;
        return Mode::Descent;
    }

    void eject(Body& b, const Payload* p) {
        std::vector<PlacedComponent> stay, go;
        for (const auto& part : b.parts) (isDescendantOf(part.component, p) ? go : stay).push_back(part);
        if (go.empty() || stay.empty()) return;
        emit(b, EventType::Ejection, p->name(), "ejection of " + p->name());
        StateVec y = b.y;
        const Quat q = b.snap.attitude;
        const Vec3 dv = q.rotate(Vec3::unitX()) * p->ejectionSpeed();
        setVec(y, 3, velOf(y) + dv);
        auto child = makeBody(p->name(), b.name, go, b.t, y, Mode::Descent, b.events, false);
        child->apogee = b.apogee;
        child->liftoff = true;
        child->railExit = true;
        child->maxAltitude = b.maxAltitude;
        if (b.hasApogeePosition) {
            child->apogeePosition = b.apogeePosition;
            child->hasApogeePosition = true;
        }
        pending_.push_back(std::move(child));
        b.parts = std::move(stay);
        rebuild(b, false);
    }

    void separate(Body& b, const Separation& s) {
        const int k = rocket_.stackIndexOf(s.aftSectionStart);
        std::vector<PlacedComponent> fwd, aft;
        for (const auto& part : b.parts) (part.stackIndex < k ? fwd : aft).push_back(part);
        if (fwd.empty() || aft.empty()) return;
        emit(b, EventType::Separation, s.name, "separation '" + s.name + "'");
        const Quat q = b.snap.attitude;
        const Vec3 axis = q.rotate(Vec3::unitX());
        StateVec yf = b.y;
        setVec(yf, 3, velOf(b.y) + axis * (0.5 * s.relativeSpeed));
        setVec(b.y, 3, velOf(b.y) - axis * (0.5 * s.relativeSpeed));
        const std::string fwdName = s.forwardName.empty() ? s.name + " forward" : s.forwardName;
        auto child = makeBody(fwdName, b.name, fwd, b.t, yf, Mode::Descent, b.events, false);
        child->apogee = b.apogee;
        child->liftoff = true;
        child->railExit = true;
        child->maxAltitude = b.maxAltitude;
        if (b.hasApogeePosition) {
            child->apogeePosition = b.apogeePosition;
            child->hasApogeePosition = true;
        }
        // Hand over runtime states of the forward components.
        for (auto& r : b.recovery)
            for (auto& cr : child->recovery)
                if (cr.device == r.device) cr = r;
        for (auto& m : b.motors)
            for (auto& cm : child->motors)
                if (cm.mount == m.mount) cm = m;
        child->mode = modeForSplitBody(*child);
        pending_.push_back(std::move(child));
        b.parts = std::move(aft);
        if (!s.aftName.empty() && s.aftName != b.name) {
            // The aft section keeps the history of the vehicle under its new name.
            const std::string old = b.name;
            EventLog renamed;
            for (FlightEvent e : b.events.all()) {
                if (e.body == old) e.body = s.aftName;
                renamed.add(e);
            }
            b.events = renamed;
            b.name = s.aftName;
            b.result.name = s.aftName;
            for (auto& p : pending_)
                if (p->parent == old) {
                    p->parent = s.aftName;
                    p->result.parent = s.aftName;
                }
        }
        rebuild(b, false);
        if (b.mode != Mode::Constrained) b.mode = modeForSplitBody(b);
    }

    void emit(Body& b, EventType type, const std::string& source, const std::string& message) {
        FlightEvent e{b.t, type, b.name, source, message, b.y[2], velOf(b.y).norm()};
        pushEvent(b, e);
    }

    void pushEvent(Body& b, const FlightEvent& e) {
        b.events.add(e);
        for (auto& o : observers_) o->onEvent(e);
    }

    void record(Body& b, const Diag& d, bool force) {
        FlightRecord r;
        r.time = b.t;
        r.phase = b.snap.phase;
        r.position = posOf(b.y);
        r.velocity = velOf(b.y);
        r.acceleration = d.accel;
        r.altitude = r.position.z;
        r.altitudeAsl = env_.site().altitude + r.position.z;
        env_.toGeodetic(r.position, r.latitude, r.longitude);
        r.speed = r.velocity.norm();
        r.verticalVelocity = r.velocity.z;
        r.horizontalSpeed = std::hypot(r.velocity.x, r.velocity.y);
        r.airspeed = d.airspeed;
        r.mach = d.mach;
        r.reynolds = d.aero.reynolds;
        r.dynamicPressure = d.q;
        r.angleOfAttack = d.aoa;
        r.thrust = d.thrust;
        r.dragForce = d.dragForce;
        r.axialForce = d.q * d.aero.referenceArea * d.aero.axial;
        r.normalForce = d.normalForce;
        r.gravity = d.g;
        r.cd = d.aero.cd;
        r.cdFriction = d.aero.cdFriction;
        r.cdPressure = d.aero.cdPressure;
        r.cdBase = d.aero.cdBase;
        r.cnAlpha = b.mode == Mode::Descent ? 0.0 : d.aero.cnAlpha;
        r.cp = d.aero.cp;
        r.cg = d.cg;
        r.stabilityMargin = d.aero.referenceLength > 0.0 && b.mode != Mode::Descent ? (d.aero.cp - d.cg) / d.aero.referenceLength : 0.0;
        r.mass = d.mass;
        r.propellantMass = d.prop;
        r.ixx = d.ixx;
        r.iyy = d.iyy;
        r.angularVelocity = b.snap.angularVelocity;
        const Vec3 ax = b.snap.attitude.rotate(Vec3::unitX());
        r.zenith = mathrix::rad2deg(std::acos(mathrix::clamp(ax.z, -1.0, 1.0)));
        double az = mathrix::rad2deg(std::atan2(ax.x, ax.y));
        r.azimuth = az < 0.0 ? az + 360.0 : az;
        r.axialAcceleration = d.specificForceBody.x;
        r.wind = d.wind;
        r.temperature = d.atm.temperature;
        r.pressure = d.atm.pressure;
        r.density = d.atm.density;
        r.speedOfSound = d.atm.speedOfSound;
        r.recoveryDragArea = d.recoveryCdA;
        r.recoveryForce = d.recoveryForce;
        for (auto& o : observers_) o->onStep(b.name, r, b.sensors->readings());
        const double interval =
            b.mode == Mode::Descent ? std::max(opt_.recordInterval, opt_.descentRecordInterval) : opt_.recordInterval;
        if (force || interval <= 0.0 || b.t - b.lastRecord >= interval - 1e-9) {
            b.result.records.push_back(r);
            b.lastRecord = b.t;
        }
    }

    void finishBody(Body& b) {
        BodyResult& res = b.result;
        res.events = b.events;
        res.mass = b.snap.mass;
        for (auto& r : b.recovery) res.recovery.push_back(r.report);
        if (b.landed && !res.records.empty()) {
            LandingReport& l = res.landing;
            const FlightRecord& last = res.records.back();
            l.landed = true;
            l.time = b.t;
            l.position = posOf(b.y);
            env_.toGeodetic(l.position, l.latitude, l.longitude);
            l.distance = std::hypot(l.position.x, l.position.y);
            double brg = mathrix::rad2deg(std::atan2(l.position.x, l.position.y));
            l.bearing = brg < 0.0 ? brg + 360.0 : brg;
            l.impactSpeed = last.speed;
            l.kineticEnergy = 0.5 * last.mass * last.speed * last.speed;
            // Mean descent rate over the last 100 m.
            for (auto it = res.records.rbegin(); it != res.records.rend(); ++it) {
                if (it->altitude >= 100.0 || it->verticalVelocity > 0.0) {
                    const double dt = l.time - it->time;
                    if (dt > 0.0) l.descentRate = it->altitude / dt;
                    break;
                }
            }
            if (l.descentRate <= 0.0) l.descentRate = -last.verticalVelocity;
            if (b.hasApogeePosition)
                l.driftFromApogee = std::hypot(l.position.x - b.apogeePosition.x, l.position.y - b.apogeePosition.y);
        }
    }
};

}  // namespace

Simulation::Simulation(Rocket rocket, Environment environment, SimulationOptions options)
    : rocket_(std::move(rocket)), environment_(std::move(environment)), options_(std::move(options)) {}

Simulation::~Simulation() = default;

Simulation::Simulation(Simulation&& o) noexcept
    : rocket_(std::move(o.rocket_)),
      environment_(std::move(o.environment_)),
      options_(std::move(o.options_)),
      observers_(std::move(o.observers_)),
      progress_(std::move(o.progress_)),
      cancel_(o.cancel_.load()) {}

Simulation& Simulation::operator=(Simulation&& o) noexcept {
    rocket_ = std::move(o.rocket_);
    environment_ = std::move(o.environment_);
    options_ = std::move(o.options_);
    observers_ = std::move(o.observers_);
    progress_ = std::move(o.progress_);
    cancel_ = o.cancel_.load();
    return *this;
}

void Simulation::addObserver(std::shared_ptr<SimulationObserver> observer) {
    if (observer) observers_.push_back(std::move(observer));
}

void Simulation::setProgressCallback(std::function<bool(double, double)> callback) { progress_ = std::move(callback); }

SimulationResult Simulation::run() {
    cancel_ = false;
    Engine engine(rocket_, environment_, options_, observers_, progress_, cancel_);
    return engine.run(*this);
}

SimulationResult simulate(const Rocket& rocket, const Environment& environment, const SimulationOptions& options) {
    Simulation sim(rocket, environment, options);
    return sim.run();
}

}  // namespace rocketup
