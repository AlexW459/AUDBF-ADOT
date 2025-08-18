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
        
        void draw2D(vector<glm::vec2> &points, vector<char> adjMatrix);

        void draw3DSingle(vector<glm::vec3> &points, vector<char> adjMatrix, float dist);

        void draw3D(vector<vector<glm::vec3>> &points, vector<vector<char>> adjMatrices, float dist);

        void clear();

    private:
        SDL_Window* window;
        SDL_Renderer* renderer;

        void draw3DMesh(vector<glm::vec3> points, vector<char> adjMatrix,  
                float distToScreen, float realScreenWidth);

        int SCREEN_WIDTH;
        int SCREEN_HEIGHT;
};