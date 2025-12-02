/*
Author: Manish Rangan, Kevin Ghobrial, Peter Samaan
Class: ECE 4122
Last Date Modified: 11/29/2025
Description: Interface for loading and rendering 3D OBJ models
*/

#ifndef OBJ_MODEL_H
#define OBJ_MODEL_H

#include <vector>
#include <string>

// Simple structures for vertex, normal, texcoord, and triangle data

struct ObjVertex
{
    float x, y, z;
};

struct ObjNormal
{
    float x, y, z;
};

struct ObjTexCoord
{
    float u, v;
};

struct ObjTriangle
{
    int vi[3]; // vertex indices
    int ti[3]; // texcoord indices
    int ni[3]; // normal indices
};

class ObjModel
{
public:
    std::vector<ObjVertex>   vertices;
    std::vector<ObjNormal>   normals;
    std::vector<ObjTexCoord> texcoords;
    std::vector<ObjTriangle> triangles;

    float scale = 1.0f;   // scale so model fits in 0.2m cube
    bool  hasTexcoords = false;
    bool  hasNormals   = false;
    
    bool loadFromFile(const std::string& path);
};

void drawObjModel(const ObjModel& model, bool useTexture);

#endif // OBJ_MODEL_H
