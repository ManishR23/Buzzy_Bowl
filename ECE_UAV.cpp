#include "ECE_UAV.h"
#include <iostream>
#include <cmath>


// constructor
ECE_UAV::ECE_UAV(float x, float y, float z)
{
    posX = x;
    posY = y;
    posZ = z;

    // initial accelerations and velocities = 0
    velX = velY = velZ = 0.0f;
    accX = accY = accZ = 0.0f;

    mass = 1.0;
    maxForcePerAxis = 10.0;
    dragCoeff = 0.05;

    // PID controllers position + velocities
    pidX = PIDController(4.0, 0.2, 2.0);
    pidY = PIDController(4.0, 0.2, 2.0);
    pidZ = PIDController(5.0, 0.3, 2.5);

    pidVx = PIDController(3.0, 0.1, 0.5);
    pidVy = PIDController(3.0, 0.1, 0.5);
    pidVz = PIDController(4.0, 0.2, 0.8);
}

//Apply PID control
void ECE_UAV::applyPIDControl()
{
    // position error (dist from target pos)
    float desiredRad = 10.0f;
    float currRad = std::sqrt(posX * posX + posY * posY + posZ * posZ);

    float error = desiredRad - currRad;

    // calculate force to maintain path
    accX = pidX.calculate(error, 0.01f) * (posX / currRad);
    accY = pidY.calculate(error, 0.01f) * (posY / currRad);
    accZ = pidZ.calculate(error, 0.01f) * (posZ / currRad);

    //reset last error
    pidX.reset();
    pidY.reset();
    pidZ.reset();
}

// check collision with another UAV and swap velocities (because elastic collision)
void ECE_UAV::checkCollision(ECE_UAV& otherUAV)
{
    float distance = std::sqrt(std::pow(posX - otherUAV.posX, 2) +
                               std::pow(posY - otherUAV.posY, 2) +
                               std::pow(posZ - otherUAV.posZ, 2));

    if (distance < 0.01f) // < 1cm = collision
    {
        // Swap velocities
        std::swap(velX, otherUAV.velX);
        std::swap(velY, otherUAV.velY);
        std::swap(velZ, otherUAV.velZ);
    }
}

// start thread
void ECE_UAV::start()
{
    uavThread = std::thread(&ECE_UAV::controlLoop, this);
}

// motion update loop (update every 10 ms)
void ECE_UAV::controlLoop()
{
    const float dt = 0.01f; // time step
    applyPIDControl();

    float forceZ = -10.0f; // gravity given from pdf

    // velocity update
    velX += accX * dt;
    velY += accY * dt;
    velZ += (accZ + forceZ / mass) * dt;

    // position update
    posX += velX * dt + 0.5f * accX * dt * dt;
    posY += velY * dt + 0.5f * accY * dt * dt;
    posZ += velZ * dt + 0.5f * accZ * dt * dt;

    // dont want to go below z = 0
    if (posZ < 0.0f)
    {
        posZ = 0.0f;
        if (velZ < 0.0f)
        {
            velZ = 0.0f; // stop downward velocity
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // run every 10 ms
    
}

//handle collisions between all UAVs
void handleCollisions(std::vector<ECE_UAV>& uavs)
{
    for (size_t i = 0; i < uavs.size(); ++i)
    {
        for (size_t j = i + 1; j < uavs.size(); ++j)
        {
            uavs[i].checkCollision(uavs[j]);
        }
    }
}