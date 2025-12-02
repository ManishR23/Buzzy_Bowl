/*
Author: Manish Rangan, Kevin Ghobrial, Peter Samaan
Class: ECE 4122
Last Date Modified: 11/28/2025
Description: Implementation of ECE_UAV class member functions for UAV simulation.
*/

#include "ECE_UAV.h"
#include <iostream>
#include <cmath>


// constructor
ECE_UAV::ECE_UAV(float x, float y, float z)
{
    posX = x;
    posY = y;
    posZ = z;

    velX = velY = velZ = 0.0f;
    accX = accY = accZ = 0.0f;

    mass = 1.0;
    maxForcePerAxis = 20.0;
    dragCoeff = 0.05;

    pidX = PIDController(4.0, 0.2, 2.0);
    pidY = PIDController(4.0, 0.2, 2.0);
    pidZ = PIDController(5.0, 0.3, 2.5);

    pidVx = PIDController(3.0, 0.1, 0.5);
    pidVy = PIDController(3.0, 0.1, 0.5);
    pidVz = PIDController(4.0, 0.2, 0.8);

    phase = FlightPhase::OnGround;
    hasVisitedCenter = false;
    simTime = 0.0f;
}


//Apply PID control
void ECE_UAV::applyPIDControl()
{
    const float dt = 0.01f; // time step

    // sphere center (middle of field in our world coords)
    const float cx = 25.0f;
    const float cy = 50.0f;
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

// Drive UAV toward the center point (0,0,50) before getting on the sphere
void ECE_UAV::applyCenterControl()
{
    const float dt = 0.01f;

    const float cx = 25.0f;
    const float cy = 50.0f;
    const float cz = 50.0f;


    float errorX = cx - posX;
    float errorY = cy - posY;
    float errorZ = cz - posZ;

    // Reuse same PID controllers
    float forceX = static_cast<float>(pidX.calculate(errorX, dt));
    float forceY = static_cast<float>(pidY.calculate(errorY, dt));
    float forceZ = static_cast<float>(pidZ.calculate(errorZ, dt));

    // Drag
    float dragX = -static_cast<float>(dragCoeff) * velX;
    float dragY = -static_cast<float>(dragCoeff) * velY;
    float dragZ = -static_cast<float>(dragCoeff) * velZ;

    forceX += dragX;
    forceY += dragY;
    forceZ += dragZ;

    // Clamp
    if (forceX > maxForcePerAxis)  forceX = static_cast<float>(maxForcePerAxis);
    if (forceX < -maxForcePerAxis) forceX = static_cast<float>(-maxForcePerAxis);
    if (forceY > maxForcePerAxis)  forceY = static_cast<float>(maxForcePerAxis);
    if (forceY < -maxForcePerAxis) forceY = static_cast<float>(-maxForcePerAxis);
    if (forceZ > maxForcePerAxis)  forceZ = static_cast<float>(maxForcePerAxis);
    if (forceZ < -maxForcePerAxis) forceZ = static_cast<float>(-maxForcePerAxis);

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
    const float dt = 0.01f;
    const float gravity = -10.0f;

    // advance per-UAV time
    simTime += dt;

    // Decide phase
    if (simTime < 5.0f)
    {
        phase = FlightPhase::OnGround;

        std::lock_guard<std::mutex> lock(uavMutex);
        // sit on ground, no motion
        velX = velY = velZ = 0.0f;
        accX = accY = accZ = 0.0f;
        posZ = 0.0f;
    }
    else
    {
        const float cx = 25.0f, cy = 50.0f, cz = 50.0f;

        // Distance to center
        float dx = posX - cx;
        float dy = posY - cy;
        float dz = posZ - cz;
        float distToCenter = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Mark that we've been within 10m of the center
        if (!hasVisitedCenter && distToCenter <= 10.0f)
        {
            hasVisitedCenter = true;
        }

        // Choose control mode
        if (!hasVisitedCenter)
        {
            phase = FlightPhase::ToCenter;
            applyCenterControl();
        }
        else
        {
            phase = FlightPhase::OnSphere;
            applyPIDControl(); // your sphere controller
        }

        // Physics update
        std::lock_guard<std::mutex> lock(uavMutex);

        velX += accX * dt;
        velY += accY * dt;
        velZ += (accZ + gravity) * dt;

        // Clamp speed for flight phases
        float speed = std::sqrt(velX * velX + velY * velY + velZ * velZ);
        if (phase != FlightPhase::OnGround && speed > 0.0001f)
        {
            const float maxSpeed = 10.0f;
            const float minSpeed = 2.0f;

            if (speed > maxSpeed)
            {
                float s = maxSpeed / speed;
                velX *= s;
                velY *= s;
                velZ *= s;
            }
            else if (speed < minSpeed)
            {
                float s = minSpeed / speed;
                velX *= s;
                velY *= s;
                velZ *= s;
            }
        }

        posX += velX * dt + 0.5f * accX * dt * dt;
        posY += velY * dt + 0.5f * accY * dt * dt;
        posZ += velZ * dt + 0.5f * accZ * dt * dt;

        // ground clamp
        if (posZ < 0.0f)
        {
            posZ = 0.0f;
            if (velZ < 0.0f)
                velZ = 0.0f;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}


//handle collisions between all UAVs
void handleCollisions(std::vector<ECE_UAV>& uavs)
{
    std::lock_guard<std::mutex> lock(uavMutex);

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