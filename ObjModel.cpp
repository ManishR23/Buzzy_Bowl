#include "ObjModel.h"
#include <fstream>
#include <sstream>
#include <limits>
#include <iostream>
#include <cmath>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

static void parseIndexTriplet(const std::string& token, int& vi, int& ti, int& ni)
{
    vi = ti = ni = -1;

    size_t s1 = token.find('/');
    if (s1 == std::string::npos)
    {
        // just "v"
        vi = std::stoi(token);
        return;
    }

    std::string vStr  = token.substr(0, s1);
    size_t s2 = token.find('/', s1 + 1);

    std::string vtStr, vnStr;
    if (s2 == std::string::npos)
    {
        // "v/vt"
        vtStr = token.substr(s1 + 1);
    }
    else
    {
        // "v/vt/vn" or "v//vn"
        vtStr = token.substr(s1 + 1, s2 - s1 - 1);
        vnStr = token.substr(s2 + 1);
    }

    if (!vStr.empty())  vi = std::stoi(vStr);
    if (!vtStr.empty()) ti = std::stoi(vtStr);
    if (!vnStr.empty()) ni = std::stoi(vnStr);
}

bool ObjModel::loadFromFile(const std::string& path)
{
    vertices.clear();
    normals.clear();
    texcoords.clear();
    triangles.clear();

    std::ifstream in(path.c_str());
    if (!in)
    {
        std::cerr << "Failed to open OBJ file: " << path << std::endl;
        return false;
    }

    std::vector<ObjVertex>   tempV;
    std::vector<ObjNormal>   tempN;
    std::vector<ObjTexCoord> tempT;

    std::string line;

    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string prefix;
        ss >> prefix;

        if (prefix == "v")
        {
            ObjVertex v;
            ss >> v.x >> v.y >> v.z;
            tempV.push_back(v);
        }
        else if (prefix == "vn")
        {
            ObjNormal n;
            ss >> n.x >> n.y >> n.z;
            tempN.push_back(n);
        }
        else if (prefix == "vt")
        {
            ObjTexCoord t;
            ss >> t.u >> t.v;
            tempT.push_back(t);
        }
        else if (prefix == "f")
        {
            ObjTriangle tri;
            for (int i = 0; i < 3; ++i)
            {
                std::string tok;
                ss >> tok;
                int vi, ti, ni;
                parseIndexTriplet(tok, vi, ti, ni);

                tri.vi[i] = (vi > 0) ? (vi - 1) : -1;
                tri.ti[i] = (ti > 0) ? (ti - 1) : -1;
                tri.ni[i] = (ni > 0) ? (ni - 1) : -1;
            }
            triangles.push_back(tri);
        }
    }

    if (tempV.empty())
    {
        std::cerr << "OBJ has no vertices: " << path << std::endl;
        return false;
    }

    // Compute bounding box
    float minX =  std::numeric_limits<float>::max();
    float minY =  std::numeric_limits<float>::max();
    float minZ =  std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    for (const auto& v : tempV)
    {
        if (v.x < minX) minX = v.x;
        if (v.y < minY) minY = v.y;
        if (v.z < minZ) minZ = v.z;

        if (v.x > maxX) maxX = v.x;
        if (v.y > maxY) maxY = v.y;
        if (v.z > maxZ) maxZ = v.z;
    }

    // Center
    float cx = 0.5f * (minX + maxX);
    float cy = 0.5f * (minY + maxY);
    float cz = 0.5f * (minZ + maxZ);

    // Shift vertices so model is centered at origin
    vertices.reserve(tempV.size());
    for (const auto& v : tempV)
    {
        ObjVertex vv;
        vv.x = v.x - cx;
        vv.y = v.y - cy;
        vv.z = v.z - cz;
        vertices.push_back(vv);
    }

    normals   = tempN;
    texcoords = tempT;

    hasNormals   = !normals.empty();
    hasTexcoords = !texcoords.empty();

    float dimX = maxX - minX;
    float dimY = maxY - minY;
    float dimZ = maxZ - minZ;
    float maxDim = std::max(dimX, std::max(dimY, dimZ));

    if (maxDim > 0.0f)
        scale = 0.6f / maxDim;      // 20 cm cube
    else
        scale = 1.0f;

    std::cout << "Loaded OBJ " << path
              << " (verts = " << vertices.size()
              << ", tris = "  << triangles.size()
              << "), scale = " << scale << std::endl;

    return true;
}

void drawObjModel(const ObjModel& model, bool useTexture)
{
    glBegin(GL_TRIANGLES);
    for (const auto& tri : model.triangles)
    {
        for (int i = 0; i < 3; ++i)
        {
            int vi = tri.vi[i];
            int ti = tri.ti[i];
            int ni = tri.ni[i];

            if (model.hasNormals && ni >= 0 && ni < (int)model.normals.size())
            {
                const auto& n = model.normals[ni];
                glNormal3f(n.x, n.y, n.z);
            }

            if (useTexture && model.hasTexcoords && ti >= 0 && ti < (int)model.texcoords.size())
            {
                const auto& t = model.texcoords[ti];
                glTexCoord2f(t.u, t.v);
            }

            const auto& v = model.vertices[vi];
            glVertex3f(v.x, v.y, v.z);
        }
    }
    glEnd();
}
