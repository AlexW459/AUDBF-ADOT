#include "meshWindow.h"

meshWindow::meshWindow(int _SCREEN_WIDTH, int _SCREEN_HEIGHT){
    window = NULL;
    SCREEN_WIDTH = _SCREEN_WIDTH;
    SCREEN_HEIGHT = _SCREEN_HEIGHT;
    SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN, &window, &renderer);
    clear();

    if(!window){
        throw std::runtime_error("Failed to initialise SDL window");
    }
}

meshWindow::~meshWindow(){
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}

void meshWindow::draw2D(vector<glm::vec2> &points, vector<char> adjMatrix) {
    

    //Width and position of the screen in same coordinate system as the given points
    glm::vec2 minPos = points[0];
    glm::vec2 maxPos = points[0];
    int numPoints = points.size();
    for(int i = 0; i < numPoints; i++){
        minPos[0] = min(minPos[0], points[i][0]);
        minPos[1] = min(minPos[1], points[i][1]);

        maxPos[0] = max(maxPos[0], points[i][0]);
        maxPos[1] = max(maxPos[1], points[i][1]);
    }


    //Zoom and position of camera
    float zoom;
    glm::vec2 viewPos;
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

void meshWindow::draw3DSingle(vector<glm::vec3> &points, vector<char> adjMatrix, float dist){

    //Finds bounding box of shape
    glm::vec3 minPos = points[0];
    glm::vec3 maxPos = points[0];
    int numPoints = points.size();
    for(int i = 0; i < numPoints; i++){
        minPos[0] = min(minPos[0], points[i][0]);
        minPos[1] = min(minPos[1], points[i][1]);
        minPos[2] = min(minPos[2], points[i][2]);

        maxPos[0] = max(maxPos[0], points[i][0]);
        maxPos[1] = max(maxPos[1], points[i][1]);
        maxPos[2] = max(maxPos[2], points[i][2]);
    }

    glm::vec3 translation = -1.0f*glm::vec3((maxPos[0] + minPos[0])/2, minPos[1], (maxPos[2] + minPos[2])/2);

    float yPos = dist;
    float distToScreen = 0.3;

    vector<glm::vec3> transformedPoints;
    transformedPoints.resize(numPoints);
    
    //Caps framerate
    SDL_Delay(16);

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

                }
                break;
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
            default:
            {
                //Transforms points
                for(int i = 0; i < numPoints; i++){
                    transformedPoints[i] = points[i] + translation + glm::vec3(0, 1, 0)*yPos;
                }

                draw3DMesh(transformedPoints, adjMatrix, distToScreen, 0.3);

                break;
            }
                
            }

        }
    }

}

void meshWindow::draw3DMesh(vector<glm::vec3> points, vector<char> adjMatrix,  
    float distToScreen, float realScreenWidth){

    int numPoints = points.size();
    vector<glm::ivec2> displayPoints;
    displayPoints.resize(numPoints);

    glm::vec2 realScreenDim(realScreenWidth, realScreenWidth*SCREEN_HEIGHT/SCREEN_WIDTH);


    //Transforms points
    for(int i = 0; i < numPoints; i++){
        glm::vec3 transformedPoint = points[i];
        //Perspective divide
        float screenXPos = transformedPoint[0]  * (distToScreen / transformedPoint[1]) / (realScreenDim[0]);
        float screenYPos = transformedPoint[2]  * (distToScreen / transformedPoint[1]) / (realScreenDim[1]);

        displayPoints[i][0] = (float)round(SCREEN_WIDTH * (0.5 + screenXPos));
        displayPoints[i][1] = (float)round(SCREEN_HEIGHT * (0.5 - screenYPos));
    }

    //Draws lines
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    for(int i = 0; i < numPoints; i++){
        for(int j = 0; j < numPoints; j++){
            if(adjMatrix[i*numPoints + j]){
                int x1 = displayPoints[i][0];
                int y1 = displayPoints[i][1];
                int x2 = displayPoints[j][0];
                int y2 = displayPoints[j][1];

                SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
            }
        }
    }

    SDL_RenderPresent(renderer);

}

void meshWindow::clear(){
    //Draws white to the screen
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.w = SCREEN_WIDTH;
    rect.h = SCREEN_HEIGHT;

    SDL_RenderFillRect(renderer, &rect);

}

//Draws multiple meshes at once
void meshWindow::draw3D(vector<vector<glm::vec3>>& points, vector<vector<char>> adjMatrices, float dist){


    //Finds bounding box of shapes
    glm::vec3 minPos = points[0][0];
    glm::vec3 maxPos = points[0][0];

    int numMeshes = points.size();
    for(int i = 0; i < numMeshes; i++){
        int numPoints = points[i].size();
        for(int j = 0; j < numPoints; j++){
            minPos[0] = min(minPos[0], points[i][j][0]);
            minPos[1] = min(minPos[1], points[i][j][1]);
            minPos[2] = min(minPos[2], points[i][j][2]);

            maxPos[0] = max(maxPos[0], points[i][j][0]);
            maxPos[1] = max(maxPos[1], points[i][j][1]);
            maxPos[2] = max(maxPos[2], points[i][j][2]);
        }
    }

    glm::vec3 translation = -1.0f*glm::vec3((maxPos[0] + minPos[0])/2, minPos[1], (maxPos[2] + minPos[2])/2);

    float yPos = dist;
    float distToScreen = 0.3;

    //Assigns memory to store transformed points
    vector<vector<glm::vec3>> transformedPoints;
    transformedPoints.resize(numMeshes);
    for(int i = 0; i < numMeshes; i++){
        transformedPoints[i].resize(points[i].size());
    }
    
    //Caps framerate
    SDL_Delay(16);

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

                }
                break;
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
            default:
            {
                for(int i = 0; i < numMeshes; i++){
                    //Transforms points
                    for(int j = 0; j < (int)points[i].size(); j++){
                        transformedPoints[i][j] = points[i][j] + translation + glm::vec3(0, 1, 0)*yPos;
                    }

                    draw3DMesh(transformedPoints[i], adjMatrices[i], distToScreen, 0.3);
                }

                break;
            }
                
            }

        }
    }
}




