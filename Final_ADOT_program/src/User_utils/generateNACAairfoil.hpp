#pragma once

#include <vector>
#include <iostream>
#include <glm/glm.hpp>

// Generates a NACA airfoil according to NACA values
inline std::vector<glm::dvec2> generateNACAairfoil(double maxCamberPercent, double maxCamberPosDecile,
    double maxThicknessPercent, double airfoilChord, double flapRadius, double meshRes){

    double interval = 1.0/meshRes;

    //Get actual values from NACA numbers
    double maxCamber = maxCamberPercent/100.0;
    double maxCamberPos = maxCamberPosDecile/10.0;
    double maxThickness = maxThicknessPercent/100.0;
    // Curved length is airfoil length as a fraction of the length without the flap
    double curvedLength = (airfoilChord - flapRadius)/airfoilChord;

    //Checks for flat airfoil
    if(maxCamber == 0.0){
        maxCamberPos = 0.5;
    }

    //Gets number of points, accounting for the fact that the airfoil ends in a 
    // single point at both ends
    int numX, numPoints;
    //Increases mesh resolution
    numX = floor(1.5 * curvedLength * airfoilChord * meshRes) + 1;
    numPoints = 2*numX;

    //std::cout << numPoints << " " << numX << std::endl;

    std::vector<glm::dvec2> airfoilPoints(numPoints);
    std::vector<glm::dvec2> camberPoints(numX);
    std::vector<glm::dvec2> diffXY(numX);


    for(int i = 0; i < numX; i++){
        //Gets the x value by finding the current point as a fraction of the total length
        double xVal = pow((double)(i+0.01)/(numX - 1 + 0.02), 1.8);
        //Adjusts for possible reduced length of airfoil
        xVal *= curvedLength;
        camberPoints[i][0] = xVal;

        //Gets thickness of airfoil
        double camberThickness = 5*maxThickness*(0.2969*sqrt(xVal) - 
            0.1260 * xVal - 0.3516*xVal*xVal + 0.2843*xVal*xVal*xVal
            - 0.1036*xVal*xVal*xVal*xVal);

        camberThickness = std::max(camberThickness, 0.5*interval);

        //Gets y position of centreline
            //Before max camber pos
        if(xVal < maxCamberPos){
            camberPoints[i][1] = maxCamber / (maxCamberPos*maxCamberPos) * 
            (2.0*maxCamberPos*xVal - xVal*xVal) * (xVal < maxCamberPos);
        }else{
            //After max camber pos
            camberPoints[i][1] = maxCamber / ((1.0 - maxCamberPos)*(1.0 - maxCamberPos)) * 
            ((1.0 - 2.0*maxCamberPos) + 2.0*maxCamberPos*xVal - xVal*xVal) * (xVal >= maxCamberPos);
        }

        //Gets gradient perpendicular to airfoil
            //Before max camber pos
        double perpGrad;
        if(maxCamber == 0.0){
            perpGrad = 0.0;

        }else if(xVal < maxCamberPos) {
            perpGrad = 2.0*maxCamber / (maxCamberPos*maxCamberPos) * (maxCamberPos - xVal);
        }else{
            //After max camber pos
            perpGrad = 2.0*maxCamber / ((1.0 - maxCamberPos)*(1.0 - maxCamberPos)) * (maxCamberPos-xVal);
        }

        
        //Gets distance away from camber line in each dimension for upper points
        //(distance for lower points is the negative)
        /*diffXY[i][0] = -camberThickness * (perpGrad / sqrt(1.0 + perpGrad*perpGrad));
        diffXY[i][1] = camberThickness * (1.0 / sqrt(1.0 + perpGrad*perpGrad));*/

        double theta = atan(perpGrad);

        
        diffXY[i][0] =  -camberThickness * sin(theta);
        diffXY[i][1] = camberThickness * cos(theta);

        //cout << "diff: " << diffXY[i][0] << ", " << diffXY[i][1] << endl;

    }

    double yMax = 0.0;
    double yMin = 0.0;
    //Gets airfoil points by adding thickness vectors onto camber line
    for(int i = 0; i < numX; i++){

        //Subtracts half of the airfoil length so that airfoil is centred around the origin
        airfoilPoints[i][0] = camberPoints[i][0] + diffXY[i][0] - curvedLength/2.0;
        airfoilPoints[i][1] = camberPoints[i][1] + diffXY[i][1];

        yMax = std::max(yMax, airfoilPoints[i][1]);

        airfoilPoints[numPoints - numX + i][0] = camberPoints[numX - i - 1][0] - diffXY[numX - i - 1][0] - curvedLength/2.0;
        airfoilPoints[numPoints - numX + i][1] = camberPoints[numX - i - 1][1] - diffXY[numX - i - 1][1];

        yMin = std::min(yMax, airfoilPoints[numPoints - numX + 1 + i][1]);

    }

    double yAvg = 0.5*(yMax + yMin);

    // Centres all points vertically around 0 and scales all points according to the airfoil chord
    for(int i = 0; i < (int) airfoilPoints.size(); i++){
        airfoilPoints[i] = (airfoilPoints[i] - glm::dvec2(0.0, yAvg)) * airfoilChord;

        //std::cout << "x: " << airfoilPoints[i][0] << " y: " << airfoilPoints[i][1] << std::endl;
    }

    return airfoilPoints;
}


inline std::vector<glm::dvec2> generateNACAairfoil(double maxCamberPercent, double maxCamberPosDecile,
    double maxThicknessPercent, double airfoilChord, double meshRes){
        // Adds a small flap radius to avoid a very thin edge at the end of the airfoil
        return generateNACAairfoil(maxCamberPercent, maxCamberPosDecile, maxThicknessPercent, 
            airfoilChord, airfoilChord*0.1, meshRes);
}