/*
Author: Manish Rangan, Kevin Ghobrial, Peter Samaan
Class: ECE 4122
Last Date Modified: 11/28/2025
Description: Interface for ECE_UAV class representing UAVs in simulation
*/

#ifndef ECE_UAV_H
#define ECE_UAV_H

#include <thread>
#include <chrono>
#include <vector>
#include <mutex>

extern std::mutex uavMutex;

class PIDController
{
public:
    double Kp, Ki, Kd;
    double integral, lastError;

    // constructor
    PIDController(double p = 0.0, double i = 0.0, double d = 0.0)
        : Kp(p), Ki(i), Kd(d), integral(0.0), lastError(0.0) {}
    
    // calculate PID output
    double calculate(double error, double dt)
    {
        integral += error * dt;
        double derivative = (dt > 0.0) ? (error - lastError) / dt : 0.0;
        lastError = error;

        return Kp * error + Ki * integral + Kd * derivative;
    }

    void reset()
    {
        integral = 0.0;
        lastError = 0.0;
    }
};

class ECE_UAV
{
public:
    
    // UAV variables
    float posX, posY, posZ;
    float velX, velY, velZ;
    float accX, accY, accZ;

    double mass;
    double maxForcePerAxis;
    double dragCoeff;

    // PID Controller variables
    PIDController pidX, pidY, pidZ;
    PIDController pidVx, pidVy, pidVz;

    // constructor
    ECE_UAV(float x, float y, float z);

    // methods
    void applyPIDControl();
    void checkCollision(ECE_UAV& otherUAV);
    void controlLoop();
};

void threadFunction(ECE_UAV* uav);
void handleCollisions(std::vector<ECE_UAV>& uavs);

#endif