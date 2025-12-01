/*
Author: Manish Rangan, Kevin Ghobrial, Peter Samaan
Class: ECE 4122
Last Date Modified: 11/28/2025
Description: Simulates 15 UAVs on a virtual football field using OpenGL. 
Each UAV is represented by an instance of the ECE_UAV class and is managed
in its own thread, allowing concurrent updates to position and behavior.
*/

#include "ECE_UAV.h"
#include <iostream>
#include <vector>
#include <GL/glut.h>
#include <thread>
#include <mutex>


// global UAVs vector
std::vector<ECE_UAV> uavs;
std::mutex uavMutex;

// init UAVs onto football field
void initUAVs()
{
    float uavPositions[15][2] = 
    {
        {0, 0}, {0, 25}, {0, 50}, {0, 75}, {0, 100},
        {25, 0}, {25, 25}, {25, 50}, {25, 75}, {25, 100},
        {50, 0}, {50, 25}, {50, 50}, {50, 75}, {50, 100}
    };

    for (int i = 0; i < 15; ++i)
    {
        uavs.emplace_back(uavPositions[i][0], uavPositions[i][1], 0.0f);
    }
}


// OpenGL initialization
void initOpenGL()
{
    // green background for football field
    glClearColor(0.0, 0.5, 0.0, 1.0); 

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1.0, 1.0, 500.00);

    glMatrixMode(GL_MODELVIEW);
}

// display OpenGL
void display() 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // set camera
    gluLookAt(50.0, 50.0, 200.0, 
        50.0, 50.0, 0.0, 
        0.0, 1.0, 0.0);

    // UAVs = red spheres for now
    glColor3f(1.0, 0.0, 0.0);

    // lock mutex for thread safety
    uavMutex.lock();
    for (const auto& uav : uavs)
    {
        glPushMatrix();
        glTranslatef(uav.posX, uav.posY, uav.posZ);
        glutSolidSphere(2.0, 20, 20); // UAV represented as sphere
        glPopMatrix();
    }
    uavMutex.unlock();

    glutSwapBuffers();
}

// refresh display every 10 ms
void updateScene(int value)
{   
    glutPostRedisplay(); 
    glutTimerFunc(10, updateScene, 0); 
}


// main function
int main(int argc, char** argv)
{
    // initialize UAVs
    initUAVs();

    std::vector<std::thread> threads;
    threads.reserve(15);

    for (int i = 0; i < 15; ++i)
    {
        threads.emplace_back(threadFunction, &uavs[i]);
    }

    for (auto& th : threads)
    {
        th.detach(); // detach threads to run independently
    }

    // initialize OpenGL
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(400, 400);
    glutCreateWindow("Buzzy_Bowl UAV Simulation");

    initOpenGL();

    // set display function
    glutDisplayFunc(display);


    glutTimerFunc(10, updateScene, 0); // start update loop

    // start main loop
    glutMainLoop();
    return 0;
}