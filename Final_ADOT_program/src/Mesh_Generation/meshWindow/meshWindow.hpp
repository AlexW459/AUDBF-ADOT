#pragma once

#include <stdexcept>
#include <vector>
#include <iostream>
#include "SDL2/SDL.h"
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"
#include "../../../compile_config.h"

#ifdef USE_SDL

class meshWindow{
    public:

    inline meshWindow(int _SCREEN_WIDTH, int _SCREEN_HEIGHT){
        window = NULL;
        SCREEN_WIDTH = _SCREEN_WIDTH;
        SCREEN_HEIGHT = _SCREEN_HEIGHT;
        SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN, &window, &renderer);
        clear();

        if(!window){
            throw std::runtime_error("Failed to initialise SDL window");
        }
    }
    
    inline ~meshWindow(){
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
    }
        
        
    inline void draw2D(const std::vector<glm::dvec2> &points, const std::vector<char> adjMatrix) {
        
        //Width and position of the screen in same coordinate system as the given points
        glm::dvec2 minPos = points[0];
        glm::dvec2 maxPos = points[0];
        int numPoints = points.size();
        for(int i = 0; i < numPoints; i++){
            minPos[0] = std::min(minPos[0], points[i][0]);
            minPos[1] = std::min(minPos[1], points[i][1]);

            maxPos[0] = std::max(maxPos[0], points[i][0]);
            maxPos[1] = std::max(maxPos[1], points[i][1]);
        }


        //Zoom and position of camera
        double zoom;
        glm::dvec2 viewPos;
        if((maxPos[0] - minPos[0])/SCREEN_WIDTH > (maxPos[1] - minPos[1])/SCREEN_HEIGHT){
            zoom = SCREEN_WIDTH/(maxPos[0] - minPos[0])*0.75;
        }else{
            zoom =  SCREEN_HEIGHT/(maxPos[1] - minPos[1])*0.75;
        }
        
        viewPos[0] = (maxPos[0] + minPos[0])*0.5;
        viewPos[1] = (maxPos[1] + minPos[1])*0.5;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        for(int i = 0; i < numPoints; i++){
            for(int j = 0; j < numPoints; j++){
                if(adjMatrix[i*numPoints + j]){
                    int x1 = SCREEN_WIDTH/2 + points[i][0]*zoom-viewPos[0]*zoom;
                    int y1 = SCREEN_HEIGHT/2-points[i][1]*zoom+viewPos[1]*zoom;
                    int x2 = SCREEN_WIDTH/2 + points[j][0]*zoom-viewPos[0]*zoom;
                    int y2 = SCREEN_HEIGHT/2-points[j][1]*zoom+viewPos[1]*zoom;

                    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
                }
            }
        }

        SDL_RenderPresent(renderer);


        //Waits for the escape key to be pressed or the window to be closed before the program continues
        SDL_Event event;
        bool windowOpen = true;
        while(windowOpen){
            //Checks if any buttons have been pressed
            while (SDL_PollEvent(&event)){
                switch(event.type){
                case SDL_KEYDOWN:
                {
                    if(event.key.keysym.sym == SDLK_ESCAPE){
                        windowOpen = false;
                        break;
                    }
                }
                case SDL_QUIT:
                {
                    windowOpen = false;
                    break;
                }
                case SDL_KEYUP:
                {
                    break;
                }
                    
                }

            }
        }
    }

    //void draw3DSingle(std::vector<glm::dvec3> &points, std::vector<char> adjMatrix, 
    //    double dist, double distToScreen);

    //void draw3D(std::vector<std::vector<glm::dvec3>> &points, 
    //    std::vector<std::vector<char>> adjMatrices, double dist, double distToScreen);

    inline void clear(){
        //Draws white to the screen
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect rect;
        rect.x = 0;
        rect.y = 0;
        rect.w = SCREEN_WIDTH;
        rect.h = SCREEN_HEIGHT;

        SDL_RenderFillRect(renderer, &rect);

    }

    private:
        SDL_Window* window;
        SDL_Renderer* renderer;

        void draw3DMesh(std::vector<glm::dvec3> points, std::vector<char> adjMatrix,  
                double distToScreen, double realScreenWidth);

        int SCREEN_WIDTH;
        int SCREEN_HEIGHT;
};

#endif
