#ifndef PARACHUTE_HPP
#define PARACHUTE_HPP

class Parachute {
public:
    Parachute(double deployAltitude, double dragCoefficient);

    void deploy();
    bool isDeployed() const;
    double getDragCoefficient() const;

private:
    double deployAltitude;
    double dragCoefficient;
    bool deployed;
};

#endif // PARACHUTE_HPP