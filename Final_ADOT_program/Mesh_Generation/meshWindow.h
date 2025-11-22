#pragma once

#include <stdexcept>
#include <vector>
#include <iostream>
#include "SDL2/SDL.h"
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"

using namespace std;

class meshWindow{
    public:
        meshWindow(int _SCREEN_WIDTH, int _SCREEN_HEIGHT);
        ~meshWindow();
        
        void draw2D(vector<glm::dvec2> &points, vector<char> adjMatrix);

        void draw3DSingle(vector<glm::dvec3> &points, vector<char> adjMatrix, double dist);

        void draw3D(vector<vector<glm::dvec3>> &points, vector<vector<char>> adjMatrices, double dist);

        void clear();

    private:
        SDL_Window* window;
        SDL_Renderer* renderer;

        void draw3DMesh(vector<glm::dvec3> points, vector<char> adjMatrix,  
                double distToScreen, double realScreenWidth);

        int SCREEN_WIDTH;
        int SCREEN_HEIGHT;
};