/*
Author: Manish Rangan, Kevin Ghobrial, Peter Samaan
Class: ECE 4122
Last Date Modified: 11/29/2025
Description: Simulates 15 UAVs on a virtual football field using OpenGL. 
Each UAV is represented by an instance of the ECE_UAV class and is managed
in its own thread, allowing concurrent updates to position and behavior.
*/

#include "ECE_UAV.h"
#include <iostream>
#include <vector>
#ifdef __APPLE__
#include <GLUT/glut.h>   // macOS
#else
#include <GL/glut.h>     // Windows / Linux
#endif
#include <thread>
#include <mutex>
#include <fstream>
#include <vector>
#include "ObjModel.h"


GLuint fieldTexture = 0;
bool   fieldTextureLoaded = false;

GLuint uavTexture = 0;
bool   uavTextureLoaded = false;

ObjModel gUAVModel;
bool     gUAVModelLoaded = false;

// Simple struct to hold BMP data
struct BMPImage
{
    unsigned int width;
    unsigned int height;
    std::vector<unsigned char> data; // BGR 24-bit
};

// global uavs vector
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

// Thread function to load BMP file for each UAV
bool loadBMP(const char* filename, BMPImage& image)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file)
    {
        std::cerr << "Could not open BMP file: " << filename << std::endl;
        return false;
    }

    unsigned char header[54];
    file.read(reinterpret_cast<char*>(header), 54);

    // Very basic sanity check
    if (header[0] != 'B' || header[1] != 'M')
    {
        std::cerr << "Not a valid BMP file: " << filename << std::endl;
        return false;
    }

    unsigned int dataPos   = *reinterpret_cast<unsigned int*>(&header[0x0A]);
    unsigned int imageSize = *reinterpret_cast<unsigned int*>(&header[0x22]);
    unsigned int width     = *reinterpret_cast<unsigned int*>(&header[0x12]);
    unsigned int height    = *reinterpret_cast<unsigned int*>(&header[0x16]);

    if (imageSize == 0)
        imageSize = width * height * 3; // 3 bytes per pixel (RGB)
    if (dataPos == 0)
        dataPos = 54;

    image.width  = width;
    image.height = height;
    image.data.resize(imageSize);

    file.seekg(dataPos, std::ios::beg);
    file.read(reinterpret_cast<char*>(image.data.data()), imageSize);

    if (!file)
    {
        std::cerr << "Error reading BMP pixel data from " << filename << std::endl;
        return false;
    }

    return true;
}

// Create OpenGL texture from BMP data for UAVs
void loadUAVTexture()
{
    BMPImage img;
    if (!loadBMP("uav.bmp", img))
    {
        std::cerr << "Could not load uav.bmp – UAVs will be untextured.\n";
        uavTextureLoaded = false;
        return;
    }

    glGenTextures(1, &uavTexture);
    glBindTexture(GL_TEXTURE_2D, uavTexture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 img.width, img.height, 0,
                 GL_BGR, GL_UNSIGNED_BYTE, img.data.data());

    uavTextureLoaded = true;
    std::cout << "Loaded UAV texture from uav.bmp ("
              << img.width << "x" << img.height << ")\n";
}

// Create OpenGL texture from BMP data for football field
void loadFieldTexture()
{
    BMPImage img;
    if (!loadBMP("ff.bmp", img))
    {
        std::cerr << "Using solid-color field (texture failed)." << std::endl;
        fieldTextureLoaded = false;
        return;
    }

    glGenTextures(1, &fieldTexture);
    glBindTexture(GL_TEXTURE_2D, fieldTexture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // BMP data is in BGR format
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB,
        img.width,
        img.height,
        0,
        GL_BGR,             // If this gives an error, try GL_BGR_EXT
        GL_UNSIGNED_BYTE,
        img.data.data()
    );

    fieldTextureLoaded = true;
    std::cout << "Loaded field texture from ff.bmp (" << img.width
              << "x" << img.height << ")" << std::endl;
}

// OpenGL initialization
void initOpenGL()
{
    glClearColor(0.0f, 0.5f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1.0, 1.0, 500.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_NORMALIZE);        // keeps normals valid when scaled

    loadFieldTexture();            // field texture
    loadUAVTexture();              // 🚨 extra-credit UAV texture

    gUAVModelLoaded = gUAVModel.loadFromFile("OBJ/simple_uav.obj");
}

// display OpenGL
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera at bird's-eye view
    gluLookAt(
        25.0, 50.0, 180.0,   // eye position (x,y,z)
        25.0, 50.0, 0.0,     // look-at point on the field
        0.0, 1.0, 0.0        // up direction
    );

    // Draw football field as textured quad
    if (fieldTextureLoaded)
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, fieldTexture);
        glColor3f(1.0f, 1.0f, 1.0f);   // don't tint the texture

        // Field extents: cover the region of your UAV positions
        float xMin = -10.0f;
        float xMax =  60.0f;
        float yMin = -10.0f;
        float yMax = 110.0f;
        float z    =  0.0f;

        glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 0.0f); glVertex3f(xMin, yMin, z);
            glTexCoord2f(1.0f, 0.0f); glVertex3f(xMax, yMin, z);
            glTexCoord2f(1.0f, 1.0f); glVertex3f(xMax, yMax, z);
            glTexCoord2f(0.0f, 1.0f); glVertex3f(xMin, yMax, z);
        glEnd();
    }
    else
    {
        // Fallback solid green field
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.0f, 0.6f, 0.0f);
        glBegin(GL_QUADS);
            glVertex3f(-10.0f, -10.0f, 0.0f);
            glVertex3f( 60.0f, -10.0f, 0.0f);
            glVertex3f( 60.0f, 110.0f, 0.0f);
            glVertex3f(-10.0f, 110.0f, 0.0f);
        glEnd();
    }

    // Draw UAVs as red spheres
    glDisable(GL_TEXTURE_2D);                // important: no texture on UAVs

    {
        std::lock_guard<std::mutex> lock(uavMutex);

        for (const auto& uav : uavs)
        {
            glPushMatrix();

            // lift slightly above ground so they don't z-fight with the field
            float drawZ = uav.posZ + 1.0f;
            glTranslatef(uav.posX, uav.posY, drawZ);

            if (gUAVModelLoaded)
            {
                // Scale so the model fits in a 20-cm cube
                float s = gUAVModel.scale* 20.0f;
                glScalef(s, s, s);

                // ---- EXTRA CREDIT: textured UAV ----
                if (uavTextureLoaded && gUAVModel.hasTexcoords)
                {
                    glEnable(GL_TEXTURE_2D);
                    glBindTexture(GL_TEXTURE_2D, uavTexture);
                    glColor3f(1.0f, 1.0f, 1.0f);   // don’t tint the texture
                    drawObjModel(gUAVModel, true); // use texture coords
                }
                else
                {
                    // Model loaded but no UAV texture / UVs → solid color
                    glDisable(GL_TEXTURE_2D);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    drawObjModel(gUAVModel, false);
                }
            }
            else
            {
                // Fallback if OBJ failed to load
                glDisable(GL_TEXTURE_2D);
                glColor3f(1.0f, 0.0f, 0.0f);
                glutSolidSphere(2.0, 20, 20);
            }

            glPopMatrix();
        }
    }

    glutSwapBuffers();
}

// Update UAV physics, check collisions, refresh display
void updateScene(int value)
{

    {
        std::lock_guard<std::mutex> lock(uavMutex);
        if (!uavs.empty())
        {
            std::cout << "UAV0 pos: (" 
                      << uavs[0].posX << ", "
                      << uavs[0].posY << ", "
                      << uavs[0].posZ << ")\n";
        }
    }

    handleCollisions(uavs);

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