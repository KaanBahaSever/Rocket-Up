#ifndef ENGINE_HPP
#define ENGINE_HPP

#include <string>
#include <vector>
#include <functional>

class Engine {
private:
    std::string engineName;
    double thrust;
    double specificImpulse;
    double massFlowRate;
    double chamberPressure;
    double exitPressure;
    double exitArea;
    double propellantMass;
    bool isActive;
    std::function<double(double)> thrustCurve;
    double burnTime;

public:
    Engine(const std::string& name, 
           double isp,
           double propMass,
           std::function<double(double)> thrustCurveFunc);

    void start();
    void shutdown();
    bool isRunning() const;

    double calculateThrust(double atmosphericPressure) const;
    double getRemainingBurnTime() const;

    double getThrust() const;
    double getIsp() const;
    double getMassFlowRate() const;
    double getPropellantMass() const;
    std::string getName() const;
    double getCurrentBurnTime() const;

    void consumePropellant(double deltaTime);

    virtual void setThrottle(double throttlePercentage);
};

#endif // ENGINE_HPP
