#include "surfaceMeshGen.h"

#pragma omp declare simd
int meshIndexTo1DIndex(int i, int j, int k, int sizeX, int sizeY) {
    return (k * sizeY + j) * sizeX + i;
}

#pragma omp declare simd
float smoothMin(float a, float b, float k){
    k *= 2.0;
    float x = b-a;
    return 0.5*( a+b-sqrt(x*x+k*k) );
}

glm::ivec3 initSDF(vector<double>& SDF, vector<glm::dvec3>& XYZ, glm::dmat2x3 totalBoundingBox, double surfMeshRes){

    //Generates matrices of values
    glm::dvec3 boundSize = totalBoundingBox[1] - totalBoundingBox[0];
    
    //Adjusts boundary to account for the fact that the resolution doesn't perfectly divide into
    //the bounding box
    double interval = 1.0/surfMeshRes;
    glm::dvec3 endBoundRemainder(fmod(boundSize[0], interval), fmod(boundSize[1], interval), fmod(boundSize[2], interval));

    totalBoundingBox[1] += glm::dvec3(interval, interval, interval) - endBoundRemainder;

    boundSize = totalBoundingBox[1] - totalBoundingBox[0];

    //Adds an additional two point2 to account for the first and last point
    glm::ivec3 SDFSize(round(boundSize / interval) + glm::dvec3(1.0, 1.0, 1.0));

    int totalSDFSize = SDFSize[0]*SDFSize[1]*SDFSize[2];

    XYZ.resize(totalSDFSize);

    //Fills meshgrids
    vector<glm::dvec3> xVals;
    xVals.resize(SDFSize[0]);
    vector<glm::dvec3> yVals;
    yVals.resize(SDFSize[1]);
    vector<glm::dvec3> zVals;
    zVals.resize(SDFSize[2]);
    #pragma omp simd
    for(int i = 0; i < SDFSize[0]; i++){
        xVals[i] = glm::dvec3(totalBoundingBox[0][0] + i*interval, 0, 0);
    }
    #pragma omp simd
    for(int i = 0; i < SDFSize[1]; i++){
        yVals[i] = glm::dvec3(0, totalBoundingBox[0][1] + i*interval, 0);
    }
    #pragma omp simd
    for(int i = 0; i < SDFSize[2]; i++){
        zVals[i] = glm::dvec3(0, 0, totalBoundingBox[0][2] + i*interval);
    }

    #pragma omp simd collapse(3)
    for(int i = 0; i < SDFSize[0]; i++){
        for(int j = 0; j < SDFSize[1]; j++){
            for(int k = 0; k < SDFSize[2]; k++){
                XYZ[meshIndexTo1DIndex(i, j, k, SDFSize[0], SDFSize[1])] = xVals[i] + yVals[j] + zVals[k];
            }
        }
    }

    //Initialises SDF by assuming all points are well outside the model
    SDF.resize(totalSDFSize);
    #pragma omp simd
    for(int i = 0; i < totalSDFSize; i++){
        SDF[i] = 50.0;

    }
    
    return SDFSize;
}


void updateSDF(vector<double>& SDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ, const vector<profile>& profiles, 
    vector<int> profileIndices, const vector<extrusionData>& extrusions, const vector<vector<int>>& parentIndices,
    vector<glm::dmat2x3> boundingBoxes, glm::dmat2x3 totalBoundingBox, double surfMeshRes){

    
    int numParts = extrusions.size();

    glm::dvec3 boundSize = totalBoundingBox[1] - totalBoundingBox[0];

    //Gets SDF of each part
    for(int p = 0; p < numParts; p++){

        //Get SDF of part
        vector<double> partSDF;
        //Finds indices of edge of bounding box of part
        glm::imat2x3 boundingIndices;
        boundingIndices[0] = floor((boundingBoxes[p][0]-totalBoundingBox[0])/boundSize*glm::dvec3(SDFSize - glm::ivec3(1, 1, 1)));
        boundingIndices[1] = ceil((boundingBoxes[p][1]-totalBoundingBox[0])/boundSize*glm::dvec3(SDFSize - glm::ivec3(1, 1, 1)));


        glm::ivec3 partSDFSize = generatePartSDF(extrusions, profiles[profileIndices[p]], p, parentIndices[p],
            XYZ, SDFSize, boundingIndices, surfMeshRes, partSDF);

    
        //Find minimum of part SDF and total SDF for points in bounding box
        #pragma omp simd collapse(3)
        for(int i = 0; i < partSDFSize[0]; i++){
            for(int j = 0; j < partSDFSize[1]; j++){
                for(int k = 0; k < partSDFSize[2]; k++){

                    int index = meshIndexTo1DIndex(i + boundingIndices[0][0], j + boundingIndices[0][1], k + boundingIndices[0][2],
                        SDFSize[0], SDFSize[1]);
                        
                    int partIndex = meshIndexTo1DIndex(i, j, k, partSDFSize[0], partSDFSize[1]);
                    
                    //double initialSDF = SDF[index];

                    //Vectorisable min function
                    SDF[index] = SDF[index] + (partSDF[partIndex] - SDF[index]) * (partSDF[partIndex] < SDF[index]);

                }
            }
        }

    }
    
}




glm::ivec3 generatePartSDF(const vector<extrusionData>& extrusions, const profile& partProfile, 
    int partIndex, vector<int> parentIndices, const vector<glm::dvec3>& meshGrid, glm::ivec3 SDFSize, 
    glm::imat2x3 boundingIndices, double surfMeshRes, vector<double>& SDF){


    glm::ivec3 partSDFSize = boundingIndices[1] - boundingIndices[0];
    int totalPartSDFSize = partSDFSize[0] * partSDFSize[1] * partSDFSize[2];

    //cout << "bounding indices: " << boundingIndices[0][0] << ", " << boundingIndices[0][1] << ", " << boundingIndices[0][2] << endl;
    //cout << "SDFSize: " <<  SDFSize[0] << ", " << SDFSize[1] << ", " << SDFSize[2] << endl;
    //cout << "partSDFSize: " <<  partSDFSize[0] << ", " << partSDFSize[1] << ", " << partSDFSize[2] << endl;

    //Copies coordinates from within bounding box to local variables
    vector<glm::dvec3> XYZ;
    XYZ.resize(totalPartSDFSize);


    #pragma omp simd collapse(3)
    for(int i = 0; i < partSDFSize[0]; i++){
        for(int j = 0; j < partSDFSize[1]; j++){
            for(int k = 0; k < partSDFSize[2]; k++){

                int index = meshIndexTo1DIndex(i + boundingIndices[0][0], j + boundingIndices[0][1], k + boundingIndices[0][2],
                        SDFSize[0], SDFSize[1]);
    
                int partIndex = meshIndexTo1DIndex(i, j, k, partSDFSize[0], partSDFSize[1]);

                XYZ[partIndex] = meshGrid[index];
            }
        }
    }


    //Gets transformations to be applied to coordinates
    int numParents = parentIndices.size();
    vector<glm::dvec3> translations;
    translations.resize(numParents + 1);
    vector<glm::dquat> rotations;
    rotations.resize(numParents + 1);
    vector<glm::dvec3> pivotPoints;
    pivotPoints.resize(numParents + 1);

    //Adds part transformations to list
    vector<int> transformIndices = {partIndex};
    transformIndices.insert(transformIndices.begin() + 1, parentIndices.begin(), parentIndices.end());

    for(int i = 0; i < numParents + 1; i++){
        int tIndex = transformIndices[numParents - i];
        //Pre-Adds the pivot point to the translation as the translation from the pivot point
        //to the origin is the next step in the transformation anyway
        pivotPoints[i] = extrusions[tIndex].pivotPoint;
        translations[i] = extrusions[tIndex].translation + pivotPoints[i];
        rotations[i] = glm::inverse(extrusions[tIndex].rotation);

    }

    //Adds control surface rotation if required
    if(extrusions[partIndex].isControl){
        //Rotates control surface
        glm::dquat controlRot = glm::angleAxis(extrusions[partIndex].rotateAngle, extrusions[partIndex].controlAxis);
        pivotPoints.insert(pivotPoints.begin(), extrusions[partIndex].pivotPoint);
        translations.insert(translations.begin(), extrusions[partIndex].pivotPoint);
        rotations.insert(rotations.begin(), controlRot);
    }

    //Applies transformations to coordinates
    #pragma omp simd collapse(2)
    for(int p = 0; p < numParents + 1; p++){
        for(int i = 0; i < totalPartSDFSize; i++){

            XYZ[i] -= translations[p];
            XYZ[i] = rotations[p] * XYZ[i];
            XYZ[i] += pivotPoints[p];

        }
    }

    //Precalculates tables of values related to extrusion scaling and translation along its length
    
    //Gets required bounds of table
    double minZ = XYZ[0][2];
    double maxZ = XYZ[0][2];
    for(int i = 1; i < totalPartSDFSize; i++){
        minZ += (XYZ[i][2] - minZ) * (XYZ[i][2] < minZ);
        maxZ += (XYZ[i][2] - maxZ) * (XYZ[i][2] > maxZ);
    }

    //Accounts for the fact that the length of the table in the z direction is slightly larger
    //than the bounds of the SDF due to the SDF bounds not dividing exactly by the interval
    double interval = 1/surfMeshRes;
    interval = interval/5.0;
    int numZ = ceil((maxZ - minZ)/interval) + 1;
    double endZ = maxZ - fmod(maxZ - minZ, interval) + interval;
    double tableZRange = endZ - minZ;

    vector<glm::dvec3> scaleTable;
    scaleTable.resize(numZ);
    vector<glm::dvec3> posTable;
    posTable.resize(numZ);

    
    //Inserts values into vector with the start and end values added on to either side
    vector<double> zSample;
    vector<glm::dvec3> scaleVals, posVals;//xPosVals, yPosVals;
    int numVals = extrusions[partIndex].zSampleVals.size() + 2;
    zSample.resize(numVals); 
    copy(extrusions[partIndex].zSampleVals.begin(), extrusions[partIndex].zSampleVals.end(), zSample.begin()+1);

    //Copies values into vectors
    posVals.resize(numVals);
    scaleVals.resize(numVals);
    for(int i = 1; i < numVals - 1; i++){
        scaleVals[i] = glm::dvec3(extrusions[partIndex].scaleVals[i - 1], 1.0);
        posVals[i] = glm::dvec3(extrusions[partIndex].posVals[i - 1], 0.0);
    }


    //copy(extrusions[partIndex].xPosVals.begin(), extrusions[partIndex].xPosVals.end(), xPosVals.begin()+1);
    //copy(extrusions[partIndex].yPosVals.begin(), extrusions[partIndex].yPosVals.end(), yPosVals.begin()+1);
    scaleVals[0] = scaleVals[1]; 
    scaleVals[numVals - 1] = scaleVals[numVals - 2];



    //xPosVals[0] = xPosVals[1]; xPosVals[numVals - 1] = xPosVals[numVals - 2];
    //yPosVals[0] = yPosVals[1]; yPosVals[numVals - 1] = yPosVals[numVals - 2];
    posVals[0] = posVals[1]; posVals[numVals - 1] = posVals[numVals - 2];
    

    //Accounts for extrusions in opposite direction
    if(zSample[1] > zSample[numVals-2]){
        reverse(zSample.begin(), zSample.end());
        reverse(scaleVals.begin(), scaleVals.end());
        //reverse(xPosVals.begin(), xPosVals.end());
        //reverse(yPosVals.begin(), yPosVals.end());
        reverse(posVals.begin(), posVals.end());

        //Adds actual bounds of bounding box at either end of zSample
        zSample[0] = minZ + 1e-6;
        zSample[numVals-1] = endZ - 1e-6;
    }else{
        zSample[0] = minZ - 1e-6;
        zSample[numVals-1] = endZ + 1e-6;
    }



    vector<int> segmentNums;
    segmentNums.resize(numZ);
    vector<double> zTable;
    zTable.resize(numZ);

    for(int i = 0; i < numZ; i++){
        zTable[i] = minZ + (double)i * interval;

        //Gets location of current z value in list
        segmentNums[i] = lower_bound(zSample.begin()+1, zSample.end(), zTable[i]) - zSample.begin()-1;
    }

    #pragma omp simd
    for(int i = 0; i < numZ; i++){
        //Linearly interpolates between the two points either side of the segment
        double fraction = (zTable[i] - zSample[segmentNums[i]])/(zSample[segmentNums[i] + 1] - zSample[segmentNums[i]]);
        scaleTable[i] = scaleVals[segmentNums[i]] * (1.0 - fraction) + scaleVals[segmentNums[i] + 1] * fraction;
        posTable[i] = posVals[segmentNums[i]] * (1.0 - fraction) + posVals[segmentNums[i] + 1] * fraction;
        //xPosTable[i] = xPosVals[segmentNums[i]] * (1.0f - fraction) + xPosVals[segmentNums[i] + 1] * fraction;
        //yPosTable[i] = yPosVals[segmentNums[i]] * (1.0f - fraction) + yPosVals[segmentNums[i] + 1] * fraction;

    }


    //Scales and translates coordinates along the extrusion length
    #pragma omp simd
    for(int i = 0; i < totalPartSDFSize; i++){
        //Gets segment of current point
        int segmentNum = (int)((XYZ[i][2] - minZ)/tableZRange*(double)(numZ - 1));
        //Gets values of transformation using linear interpolation
        double fraction = (XYZ[i][2] - zTable[segmentNum])/interval;
        glm::dvec3 scale = scaleTable[segmentNum] * (1.0 - fraction) + scaleTable[segmentNum + 1] * fraction;
        glm::dvec3 pos = posTable[segmentNum] * (1.0 - fraction) + posTable[segmentNum + 1] * fraction;
        //float xPos = xPosTable[segmentNum] * (1.0f - fraction) + xPosTable[segmentNum + 1] * fraction;
        // float yPos = yPosTable[segmentNum] * (1.0f - fraction) + yPosTable[segmentNum + 1] * fraction;

        //Applies transformations
        XYZ[i] = (XYZ[i] - pos)/scale;

    }


    //Precalculates table of values of distances to part
    
    //Gets useful values
    vector<glm::dvec2> v = partProfile.vertexCoords;
    int numVerts = v.size();

    //Point outside model
    glm::dvec2 pOut = glm::dvec2(200.245, 150.245);


    vector<double> a1, b1, c1, d2, l2;
    vector<glm::dvec2> v2;
    vector<glm::dvec2> diff;
    v2.resize(numVerts);
    l2.resize(numVerts);
    diff.resize(numVerts);
    a1.resize(numVerts);
    b1.resize(numVerts);
    c1.resize(numVerts);
    d2.resize(numVerts);

    for(int i = 0; i < numVerts - 1; i++){
        v2[i] = v[i+1];
    }
    v2[numVerts - 1] = v[0];

    #pragma omp simd 
    for(int i = 0; i < numVerts; i++){
        diff[i] = v2[i] - v[i];
        l2[i] = sqrt(glm::dot(diff[i], diff[i]));


        a1[i] = v2[i][1] - v[i][1];
        b1[i] = v[i][0] - v2[i][0];
        c1[i] = v2[i][0] * v[i][1] - v[i][0] * v2[i][1];

        d2[i] = a1[i] * pOut[0] + b1[i] * pOut[1] + c1[i];
    }


    //Gets bounds of plane. Must be floats to avoid errors when operating with XYZ
    double minX = XYZ[0][0];
    double maxX = XYZ[0][0];
    double minY = XYZ[0][1];
    double maxY = XYZ[0][1];


    #pragma omp simd
    for(int i = 1; i < totalPartSDFSize; i++){
        minX += (XYZ[i][0] - minX) * (XYZ[i][0] < minX);
        maxX += (XYZ[i][0] - maxX) * (XYZ[i][0] > maxX);
        minY += (XYZ[i][1] - minY) * (XYZ[i][1] < minY);
        maxY += (XYZ[i][1] - maxY) * (XYZ[i][1] > maxY);
    }


    //Gets size of table given resolution
    glm::ivec2 numXY = glm::ivec2(ceil((maxX - minX)/interval) + 1, ceil((maxY - minY)/interval) + 1);
    glm::dvec2 endXY = glm::dvec2(maxX - fmod(maxX - minX, interval) + interval, maxY - fmod(maxY - minY, interval) + interval);
    glm::dvec2 tableXYRange = endXY - glm::dvec2(minX, minY);


    vector<double> SDFTable;
    int tableSize = numXY[0]*numXY[1];

    SDFTable.resize(tableSize);


    vector<double> dist2s;
    vector<double> intersections;
    dist2s.resize(numVerts);
    intersections.resize(numVerts);


    vector<double> xTable;
    vector<double> yTable;
    xTable.resize(numXY[0]);
    yTable.resize(numXY[1]);


    //Gets x and y values
    for(int x = 0; x < numXY[0]; x++){
        xTable[x] = minX + x * interval;
    }
    for(int y = 0; y < numXY[1]; y++){
        yTable[y] = minY + y * interval;
    }

    
    double minProfileX = v[0][0];
    for(int i = 1; i < numVerts; i++){
        minProfileX = min(minProfileX, v[i][0]);
    }


    //Fills table with values
    for(int x = 0; x < numXY[0]; x++){
        for(int y = 0; y < numXY[1]; y++){
            double xVal = xTable[x];
            double yVal = yTable[y];


            //Loops over each edge of the profile and checks the distance
            #pragma omp simd
            for(int e = 0; e < numVerts; e++){


                double dotProd = (xVal - v[e][0])*diff[e][0] + (yVal - v[e][1])*diff[e][1];
                
                double t = dotProd/l2[e];
                //Limits t to between 0 and 1
                t -= t * (0 > t);
                t += (1 - t) * (1 < t);

                double projx = v[e][0] + t*diff[e][0];
                double projy = v[e][1] + t*diff[e][1];
                double distx = xVal - projx;
                double disty = yVal - projy;

                //Finds distance squared and sign
                dist2s[e] = distx*distx + disty*disty;

                //Gets intersection with line segment between point and outside point
                /*If d1 and d2 have the same sign, their product is positive and so
                the result of the comparison is 0. This indicates that */
                double d1 = a1[e] * xVal + b1[e] * yVal + c1[e];
                int int1 = (d1*d2[e]) < 0.0f;

                ///Find parameters of implicit equation of line going to outside of %polygon
                double a2 = pOut[1] - yVal;
                double b2 = xVal - pOut[0];
                double c2 = pOut[0]*yVal - xVal*pOut[1];

                double d12 = a2*v[e][0] + b2*v[e][1] + c2;
                double d22 = a2*v2[e][0] + b2*v2[e][1] + c2;

                int int2 = (d12*d22) < 0.0f;

                //Determines if an intersection has occured. Both int1 and int2 must be 1
                //in order for there to have been an intersection. 
                intersections[e] = int1*int2;

            }

            //Gets total intersection count
            int totIntersections = 0;
            #pragma omp simd reduction(+:totIntersections)
            for(int e = 0; e < numVerts; e++){
                totIntersections += intersections[e];
            }

            //Gets minimum distance
            double minDist2 = dist2s[0];
            for(int e = 1; e < numVerts; e++){
                minDist2 += (dist2s[e] - minDist2) * (dist2s[e] < minDist2);
            }

            //Assigns distance with sign
            SDFTable[x*numXY[1] + y] = sqrt(minDist2) * (1 - 2*(totIntersections % 2));

        }
    }


    //Finds SDF values for profile
    vector<double> profileSDF;
    profileSDF.resize(totalPartSDFSize);

    #pragma omp simd
    for(int i = 0; i < totalPartSDFSize; i++){
        //Gets location in table
        int segmentNumX = (int)((XYZ[i][0] - minX)/tableXYRange[0] * (double)(numXY[0] - 1));
        int segmentNumY = (int)((XYZ[i][1] - minY)/tableXYRange[1] * (double)(numXY[1] - 1));

        //Linear interpolation

        double Q00 = SDFTable[segmentNumX*numXY[1] + segmentNumY];
        double Q10 = SDFTable[(segmentNumX + 1)*numXY[1] + segmentNumY];
        double Q01 = SDFTable[segmentNumX*numXY[1] + segmentNumY + 1];
        double Q11 = SDFTable[(segmentNumX + 1)*numXY[1] + segmentNumY + 1];
        
        double fractionX = (xTable[segmentNumX] - XYZ[i][0])/interval;
        double fractionY = (yTable[segmentNumY] - XYZ[i][1])/interval;

        double SDFy0 = Q00 * (1 - fractionX) + Q10 * fractionX;
        double SDFy1 = Q01 * (1 - fractionX) + Q11 * fractionX;

        profileSDF[i] = SDFy0 * (1 - fractionY) + SDFy1 * fractionY;

    }

    //Gets SDF of the faces on either end of the extrusion
    
    //Gets z locations of planes
    double beginPlaneZ = extrusions[partIndex].zSampleVals[0];
    double endPlaneZ = extrusions[partIndex].zSampleVals[0];
    for(int i = 1; i < (int)extrusions[partIndex].zSampleVals.size(); i++){
        beginPlaneZ = min(beginPlaneZ, extrusions[partIndex].zSampleVals[i]);
        endPlaneZ = max(endPlaneZ, extrusions[partIndex].zSampleVals[i]);
    }

    //Fills SDF by combining the SDF of the profile along the extrusion with the SDF of the planes on
    //either end of the extrusion
    SDF.resize(totalPartSDFSize);
    #pragma omp simd
    for(int i = 0; i < totalPartSDFSize; i++){
        //Maximum of the profile SDF and the two planes
        double beginFace = (beginPlaneZ - XYZ[i][2]) / sqrt(1 + beginPlaneZ*beginPlaneZ);
        double endFace = (XYZ[i][2] - endPlaneZ) / sqrt(1 + endPlaneZ*endPlaneZ);
        double maxFace = endFace + (beginFace - endFace) * (beginFace > endFace);



        //Assigns overall maximum value to the SDF
        SDF[i] = maxFace + (profileSDF[i] - maxFace) * (profileSDF[i] > maxFace);

    }


    return partSDFSize;
}

void applyGaussianBlur(float sigma, int n, vector<double>& SDF, glm::ivec3 SDFSize){
    int r = round(((float)n-1)/2);

    glm::ivec3 shrunkSDFSize = SDFSize - glm::ivec3(2*r, 2*r, 2*r);

    vector<float> kernelOuterRow;
    kernelOuterRow.resize(r*2+1);
    float kernelSum = 0;

    //vector describing edge row of nxnxn kernel
    for(int i = 0; i < 2*r+1; i++){
        kernelOuterRow[i] = exp(-0.5*(2*r*r + (i-r)*(i-r))/sqrt(sigma));
        kernelSum += kernelOuterRow[i];
    }

    //Divides kernel row by the cube root of its sum, which has the effect of normalising
    //the virtual matrix as a whole
    for(int i = 0; i < 2*r + 1; i++){
        kernelOuterRow[i] /= pow(kernelSum, 1.0/3.0);
    }



    //multiplies kernel with SDF in row direction
    vector<float> rowMults;
    //x direction is reduced as the row multiplication cannot be performed on elements 
    //near the edge of the SDF as they do not have a sufficient number of elements
    //To either side of them
    rowMults.resize(shrunkSDFSize[0] * SDFSize[1] * SDFSize[2]);
    #pragma omp simd collapse(3)
    for(int i = 0; i < shrunkSDFSize[0]; i++){
        for(int j = 0; j < SDFSize[1]; j++){
            for(int k = 0; k < SDFSize[2]; k++){
                rowMults[meshIndexTo1DIndex(i, j, k, shrunkSDFSize[0], SDFSize[1])] = 0.0f;
            }
        }
    }

    //x, y and z indices are with respect to the inner portion of the SDF that is not
    //trimmed during the matrix multiplication process
    #pragma omp simd collapse(4)
    for(int i = 0; i < shrunkSDFSize[0]; i++){
        for(int j = 0; j < SDFSize[1]; j++){
            for(int k = 0; k < SDFSize[2]; k++){
                for(int e = 0; e < 2*r+1; e++){
                    rowMults[meshIndexTo1DIndex(i, j, k, shrunkSDFSize[0], SDFSize[1])] +=
                        SDF[meshIndexTo1DIndex(i + e, j, k, SDFSize[0], SDFSize[1])]*kernelOuterRow[e];
                }
            }
        }
    }

    

    vector<float> colMults;
    colMults.resize(shrunkSDFSize[0] * shrunkSDFSize[1] * SDFSize[2]);
    #pragma omp simd collapse(3)
    for(int i = 0; i < shrunkSDFSize[0]; i++){
        for(int j = 0; j < shrunkSDFSize[1]; j++){
            for(int k = 0; k < SDFSize[2]; k++){
                colMults[meshIndexTo1DIndex(i, j, k, shrunkSDFSize[0], shrunkSDFSize[1])] = 0.0f;
            }
        }
    }

    //multiplies kernel with SDF in col direction
    #pragma omp simd collapse(4)
    for(int i = 0; i < shrunkSDFSize[0]; i++){
        for(int j = 0; j < shrunkSDFSize[1]; j++){
            for(int k = 0; k < SDFSize[2]; k++){
                for(int e = 0; e < 2*r+1; e++){
                    colMults[meshIndexTo1DIndex(i, j, k, shrunkSDFSize[0], shrunkSDFSize[1])] += 
                        rowMults[meshIndexTo1DIndex(i, j + e, k, shrunkSDFSize[0], SDFSize[1])]*kernelOuterRow[e];
                }
            }
        }
    }


    #pragma omp simd collapse(3)
    for(int i = r; i < shrunkSDFSize[0] + r; i++){
        for(int j = r; j < shrunkSDFSize[1] + r; j++){
            for(int k = r; k < shrunkSDFSize[2] + r; k++){
                SDF[meshIndexTo1DIndex(i, j, k, SDFSize[0], SDFSize[1])] = 0.0f;
            }
        }
    }

    #pragma omp simd collapse(4)
    for(int i = 0; i < shrunkSDFSize[0]; i++){
        for(int j = 0; j < shrunkSDFSize[1]; j++){
            for(int k = 0; k < shrunkSDFSize[2]; k++){
                for(int e = 0; e < 2*r+1; e++){
                    SDF[meshIndexTo1DIndex(i + r, j + r, k + r, SDFSize[0], SDFSize[1])] +=
                        colMults[meshIndexTo1DIndex(i, j, k + e, shrunkSDFSize[0], shrunkSDFSize[1])];
                }
            }
        }
    }



}