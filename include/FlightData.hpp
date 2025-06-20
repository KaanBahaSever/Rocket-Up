#ifndef FLIGHTDATA_HPP
#define FLIGHTDATA_HPP

#include <string>
#include "Vector3.h"

class FlightData {
public:
    // Constructor
    FlightData();

    // Destructor
    ~FlightData();

    // Getters and Setters
    int getFlightNumber() const;
    void setFlightNumber(int flightNumber);

    std::string getDepartureTime() const;
    void setDepartureTime(const std::string& departureTime);

    std::string getArrivalTime() const;
    void setArrivalTime(const std::string& arrivalTime);

    double getFlightDuration() const;
    void setFlightDuration(double flightDuration);

    double getTicketPrice() const;
    void setTicketPrice(double ticketPrice);

    Vector3 getVelocity() const;
    void setVelocity(const Vector3& velocity);

    Vector3 getAcceleration() const;
    void setAcceleration(const Vector3& acceleration);

    double getAltitude() const;
    void setAltitude(double altitude);

    double getFuelLevel() const;
    void setFuelLevel(double fuelLevel);

    double getThrust() const;
    void setThrust(double thrust);

private:
    int flightNumber;
    std::string departureCity;
    std::string arrivalCity;
    std::string departureTime;
    std::string arrivalTime;
    double flightDuration;
    double ticketPrice;

    Vector3 velocity; // Velocity vector
    Vector3 acceleration; // Acceleration vector
    double altitude;
    double fuelLevel;
    double thrust;
};

#endif // FLIGHTDATA_HPP