#ifndef ECE_UAV_H
#define ECE_UAV_H

#include <thread>
#include <chrono>
#include <vector>
#include <GL/glut.h>

class PIDController {
public:
    double Kp, Ki, Kd;
    double integral, lastError;
    double error, derivative;

    PIDController(double p = 0.0, double i = 0.0, double d = 0.0)
        : Kp(p), Ki(i), Kd(d), integral(0.0), lastError(0.0), error(0.0), derivative(0.0) {}
    
    double calculate(double error, double dt)
    {
        integral += error * dt;
        derivative = (error - lastError) / dt;
        lastError = error;

        return Kp * error + Ki * integral + Kd * derivative;
    }

    void reset()
    {
        integral = 0.0;
        lastError = 0.0;
    }
};

class ECE_UAV {
public:
    
    // UAV variables
    float posX, posY, posZ;
    float velX, velY, velZ;
    float accX, accY, accZ;
    double mass;
    double maxForcePerAxis;
    double dragCoeff;
    std::thread uavThread; // thread for UAV control loop

    // PID Controller variables
    PIDController pidX, pidY, pidZ;
    PIDController pidVx, pidVy, pidVz;

    // constructor
    ECE_UAV(float x, float y, float z);

    // methods
    void applyPIDControl();
    void checkCollision(ECE_UAV& otherUAV);
    void start();
    void join()
    {
        if (uavThread.joinable())
            uavThread.join();
    }
    void controlLoop();
};

void handleCollisions(std::vector<ECE_UAV>& uavs);

#endif