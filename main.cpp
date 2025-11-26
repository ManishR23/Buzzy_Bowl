#include "ECE_UAV.h"
#include <iostream>
#include <vector>
#include <GL/glut.h>


// global uavs vector
std::vector<ECE_UAV> uavs;

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
        ECE_UAV newUAV(uavPositions[i][0], uavPositions[i][1], 0.0f);
        uavs.push_back(newUAV);
        newUAV.start(); // staart UAV thread
        
    }
}


// OpenGL initialization
void initOpenGL()
{
    // green background for football field (change Later)
    glClearColor(0.0, 0.5, 0.0, 1.0); 

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, 125.0f / 50.0f, 1.0f, 200.0f);
}

// display OpenGL
void display() 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // set camera
    gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // UAVs = red spheres for now
    glColor3f(1.0, 0.0, 0.0);
    for (const auto& uav : uavs)
    {
        glPushMatrix();
        glTranslatef(uav.posX, uav.posY, uav.posZ);
        glutSolidSphere(1.0, 20, 20); // UAV represented as sphere
        glPopMatrix();
    }

    glutSwapBuffers();
}


// main function
int main(int argc, char** argv)
{
    // initialize UAVs
    initUAVs();

    // initialize OpenGL
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(400, 400);
    glutCreateWindow("ECE UAV Simulation");

    initOpenGL();

    // set display function
    glutDisplayFunc(display);

    // start main loop
    glutMainLoop();

    return 0;
}