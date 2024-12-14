  #ifndef ENGINE_HPP
  #define ENGINE_HPP

  #include <string>
  #include <vector>
  #include <functional>

  class Engine {
  private:
      std::string engineName;
      double thrust;                  // Newtons
      double specificImpulse;         // seconds
      double massFlowRate;            // kg/s
      double chamberPressure;         // Pascal
      double exitPressure;            // Pascal
      double exitArea;                // m^2
      double propellantMass;          // kg
      bool isActive;
      std::function<double(double)> thrustCurve;  // Function that takes burn time and returns thrust multiplier
      double burnTime;                // Current burn time since ignition

  public:
      Engine(const std::string& name, 
           double isp,
           double propMass,
           std::function<double(double)> thrustCurveFunc)
          : engineName(name),
            specificImpulse(isp),
            propellantMass(propMass),
            isActive(false),
            thrustCurve(thrustCurveFunc),
            burnTime(0.0) {}

      // Core engine operations
      void start() { isActive = true; }
      void shutdown() { /* Solid motors cannot be shut down */ }
      bool isRunning() const { return isActive && propellantMass > 0; }

      // Performance calculations
      double calculateThrust(double atmosphericPressure) const {
          double thrustMultiplier = thrustCurve(burnTime);
          return (thrust * thrustMultiplier) + (exitPressure - atmosphericPressure) * exitArea;
      }

      double getRemainingBurnTime() const {
          return propellantMass / massFlowRate;
      }

      // Getters
      double getThrust() const { return thrust * thrustCurve(burnTime); }
      double getIsp() const { return specificImpulse; }
      double getMassFlowRate() const { return massFlowRate; }
      double getPropellantMass() const { return propellantMass; }
      std::string getName() const { return engineName; }
      double getCurrentBurnTime() const { return burnTime; }

      // Propellant consumption
      void consumePropellant(double deltaTime) {
          if (isActive && propellantMass > 0) {
              propellantMass -= massFlowRate * deltaTime;
              burnTime += deltaTime;
              if (propellantMass < 0) {
                  propellantMass = 0;
                  isActive = false;
              }
          }
      }

      // Throttling disabled for solid motors
      virtual void setThrottle(double throttlePercentage) {
          // Solid rocket motors cannot be throttled
      }
  };

  #endif // ENGINE_HPP
