#include "Engine.hpp"

Engine::Engine(const std::string& name, 
               double isp,
               double propMass,
               std::function<double(double)> thrustCurveFunc)
    : engineName(name), 
      specificImpulse(isp), 
      propellantMass(propMass), 
      thrustCurve(thrustCurveFunc), 
      isActive(false), 
      burnTime(0.0) {
    massFlowRate = thrust / (specificImpulse * 9.81);
}

void Engine::start() {
    isActive = true;
    burnTime = 0.0;
}

void Engine::shutdown() {
    isActive = false;
}

bool Engine::isRunning() const {
    return isActive;
}

double Engine::calculateThrust(double atmosphericPressure) const {
    if (!isActive) return 0.0;
    double pressureDifference = chamberPressure - atmosphericPressure;
    return thrustCurve(burnTime) + (pressureDifference * exitArea);
}

double Engine::getRemainingBurnTime() const {
    return propellantMass / massFlowRate;
}

double Engine::getThrust() const {
    return thrust;
}

double Engine::getIsp() const {
    return specificImpulse;
}

double Engine::getMassFlowRate() const {
    return massFlowRate;
}

double Engine::getPropellantMass() const {
    return propellantMass;
}

std::string Engine::getName() const {
    return engineName;
}

double Engine::getCurrentBurnTime() const {
    return burnTime;
}

void Engine::consumePropellant(double deltaTime) {
    if (!isActive) return;
    double consumedMass = massFlowRate * deltaTime;
    if (consumedMass > propellantMass) {
        propellantMass = 0.0;
        shutdown();
    } else {
        propellantMass -= consumedMass;
        burnTime += deltaTime;
    }
}

void Engine::setThrottle(double throttlePercentage) {
    thrust = thrustCurve(burnTime) * (throttlePercentage / 100.0);
    massFlowRate = thrust / (specificImpulse * 9.81);
}