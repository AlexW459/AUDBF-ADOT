

#ifdef USE_SDL

using namespace std;

/*
void meshWindow::draw3DSingle(vector<glm::dvec3> &points, vector<char> adjMatrix, double dist,
     double distToScreen){

    //Finds bounding box of shape
    glm::dvec3 minPos = points[0];
    glm::dvec3 maxPos = points[0];
    int numPoints = points.size();
    for(int i = 0; i < numPoints; i++){
        minPos[0] = min(minPos[0], points[i][0]);
        minPos[1] = min(minPos[1], points[i][1]);
        minPos[2] = min(minPos[2], points[i][2]);

        maxPos[0] = max(maxPos[0], points[i][0]);
        maxPos[1] = max(maxPos[1], points[i][1]);
        maxPos[2] = max(maxPos[2], points[i][2]);
    }

    glm::dvec3 translation = -1.0*glm::dvec3((maxPos[0] + minPos[0])/2, minPos[1], (maxPos[2] + minPos[2])/2);

    double yPos = dist;

    vector<glm::dvec3> transformedPoints(numPoints);

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
                    transformedPoints[i] = points[i] + translation + glm::dvec3(0, 1, 0)*yPos;
                }

                draw3DMesh(transformedPoints, adjMatrix, distToScreen, 0.3);

                break;
            }
                
            }

        }
    }

}


//Internl function used to draw any mesh
void meshWindow::draw3DMesh(vector<glm::dvec3> points, vector<char> adjMatrix,  
    double distToScreen, double realScreenWidth){

    int numPoints = points.size();
    vector<glm::ivec2> displayPoints(numPoints);

    glm::dvec2 realScreenDim(realScreenWidth, realScreenWidth*SCREEN_HEIGHT/SCREEN_WIDTH);


    //Transforms points
    for(int i = 0; i < numPoints; i++){
        glm::dvec3 transformedPoint = points[i];
        //Perspective divide
        float screenXPos = transformedPoint[0]  * (distToScreen / transformedPoint[1]) / (realScreenDim[0]);
        float screenYPos = transformedPoint[2]  * (distToScreen / transformedPoint[1]) / (realScreenDim[1]);

        displayPoints[i][0] = round(SCREEN_WIDTH * (0.5 + screenXPos));
        displayPoints[i][1] = round(SCREEN_HEIGHT * (0.5 - screenYPos));

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

//Draws multiple meshes at once
void meshWindow::draw3D(vector<vector<glm::dvec3>>& points, vector<vector<char>> adjMatrices, 
    double dist, double distToScreen){


    //Finds bounding box of shapes
    glm::dvec3 minPos = points[0][0];
    glm::dvec3 maxPos = points[0][0];

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

    glm::dvec3 translation = -1.0*glm::dvec3((maxPos[0] + minPos[0])/2, minPos[1], (maxPos[2] + minPos[2])/2);

    double yPos = dist;

    //Assigns memory to store transformed points
    vector<vector<glm::dvec3>> transformedPoints(numMeshes);
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
                        transformedPoints[i][j] = points[i][j] + translation + glm::dvec3(0, 1, 0)*yPos;

                    
                    }


                    draw3DMesh(transformedPoints[i], adjMatrices[i], distToScreen, 0.3);
                }

                break;
            }
                
            }

        }
    }
}*/

#endif


