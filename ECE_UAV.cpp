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
    maxForcePerAxis = 20.0;
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
    const float dt = 0.01f; // time step

    // sphere center
    const float cx = 0.0f;
    const float cy = 0.0f;
    const float cz = 50.0f;

    // vector from center to UAV
    float dx = posX - cx;
    float dy = posY - cy;
    float dz = posZ - cz;

    const float desiredRad = 10.0f;
    float currRad = std::sqrt(dx * dx + dy * dy + dz * dz);

    // edge case: at center
    if (currRad < 0.01f)
    {
        accX = 0.0f;
        accY = 0.0f;
        accZ = 2.0f; // small acceleration upwards
        return;
    }

    // unit direction center to UAV
    float ux = dx / currRad;
    float uy = dy / currRad;
    float uz = dz / currRad;

    float desX = cx + ux * desiredRad;
    float desY = cy + uy * desiredRad;
    float desZ = cz + uz * desiredRad;

    // position errors
    float errorX = desX - posX;
    float errorY = desY - posY;
    float errorZ = desZ - posZ;

    // PID on position
    float forceX = static_cast<float>(pidX.calculate(errorX, dt));
    float forceY = static_cast<float>(pidY.calculate(errorY, dt));
    float forceZ = static_cast<float>(pidZ.calculate(errorZ, dt));

    // drag force (F = -kv)
    float dragX = -static_cast<float>(dragCoeff) * velX;
    float dragY = -static_cast<float>(dragCoeff) * velY;    
    float dragZ = -static_cast<float>(dragCoeff) * velZ;


    // total forces exluding gravity 
    forceX += dragX;
    forceY += dragY;
    forceZ += dragZ;

    // clamp to maxforce
    if (forceX > maxForcePerAxis) forceX = static_cast<float>(maxForcePerAxis);
    if (forceX < -maxForcePerAxis) forceX = static_cast<float>(-maxForcePerAxis);

    if (forceY > maxForcePerAxis) forceY = static_cast<float>(maxForcePerAxis);
    if (forceY < -maxForcePerAxis) forceY = static_cast<float>(-maxForcePerAxis);

    if (forceZ > maxForcePerAxis) forceZ = static_cast<float>(maxForcePerAxis);
    if (forceZ < -maxForcePerAxis) forceZ = static_cast<float>(-maxForcePerAxis);

    // acceleration = F/m
    accX = forceX / static_cast<float>(mass);
    accY = forceY / static_cast<float>(mass);
    accZ = forceZ / static_cast<float>(mass);
}

// check collision with another UAV and swap velocities (because elastic collision)
void ECE_UAV::checkCollision(ECE_UAV& otherUAV)
{
    float dx = posX - otherUAV.posX;
    float dy = posY - otherUAV.posY;
    float dz = posZ - otherUAV.posZ;

    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 0.01f) // < 1cm = collision
    {
        // Swap velocities
        std::swap(velX, otherUAV.velX);
        std::swap(velY, otherUAV.velY);
        std::swap(velZ, otherUAV.velZ);
    }
}

// motion update loop (update every 10 ms)
void ECE_UAV::controlLoop()
{
    const float dt = 0.01f; // time step
    applyPIDControl();

    const float gravity = -10.0f; // gravity given from pdf

    // lock while updating shared UAV data
    uavMutex.lock();

    // velocity update
    velX += accX * dt;
    velY += accY * dt;
    velZ += (accZ + gravity) * dt;

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

    uavMutex.unlock();

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

// thread function for UAV control loop
void threadFunction(ECE_UAV* uav)
{
    while (true)
    {
        uav->controlLoop();
    }
}