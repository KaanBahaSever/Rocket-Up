  #pragma once
  #include <Engine.hpp>
  #include <components/Parachute.hpp>
  #include <string>
  #include <vector>
  #include <memory>

  namespace core {

  class Rocket {
  public:
      Rocket();
      Rocket(const Engine engine, const Parachute parachute);
      ~Rocket();

      // Getters
      std::string getName() const { return name_; }
      double getMass() const { return mass_; }
      double getThrust() const { return thrust_; }
      double getAltitude() const { return altitude_; }
      double getVelocity() const { return velocity_; }
      std::string getEngine() const { return engine_; }

      // Setters
      void setName(const std::string& name) { name_ = name; }
      void setMass(double mass) { mass_ = mass; }
      void setThrust(double thrust) { thrust_ = thrust; }
      void setEngine(const std::string& engine) { engine_ = engine; }

      // Flight controls
      void launch();
      void abort();
      void updatePosition(double deltaTime);
      bool isLaunched() const { return isLaunched_; }

  private:
      std::string name_;
      double mass_;        // kg
      double thrust_;      // Newtons
      double altitude_;    // meters
      double velocity_;    // m/s
      bool isLaunched_;
      std::string engine_; // engine type
    
      // Helper functions
      void calculateForces();
      void updateVelocity(double deltaTime);
  };

  } // namespace core
