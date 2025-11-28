cmake_minimum_required(VERSION 3.10)

project(ECE_UAV_Simulation)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find OpenGL and FreeGLUT libraries
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Add the source files
set(SOURCES
    main.cpp
    ECE_UAV.cpp
    ObjModel.cpp
)

# Create the executable
add_executable(uav_simulation ${SOURCES})

# Link OpenGL and FreeGLUT libraries
target_link_libraries(uav_simulation ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
 