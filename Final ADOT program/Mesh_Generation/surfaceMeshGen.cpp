#include "surfaceMeshGen.h"


glm::vec3 generateSDF(vector<float>& SDF, vector<profile>& profiles, vector<int> profileIndices,
    vector<extrusionData>& extrusions, vector<vector<int>> parentIndices, glm::mat2x3 totalBoundingBox, 
    vector<glm::mat2x3> boundingBoxes, double surfMeshRes){
    //Generates matrices of values
    glm::vec3 boundSize = totalBoundingBox[1] - totalBoundingBox[0];
    glm::ivec3 SDFSize(floor(glm::dvec3(boundSize) * surfMeshRes));
    int totalSDFSize = SDFSize[0]*SDFSize[1]*SDFSize[2];
    float interval = 1/surfMeshRes;
    vector<glm::vec3> XYZ;
    XYZ.resize(totalSDFSize);


    //Fills meshgrids
    vector<glm::vec3> xVals;
    xVals.resize(SDFSize[0]);
    vector<glm::vec3> yVals;
    yVals.resize(SDFSize[1]);
    vector<glm::vec3> zVals;
    zVals.resize(SDFSize[2]);
    #pragma omp simd
    for(int i = 0; i < SDFSize[0]; i++){
        xVals[i] = glm::vec3(i*interval, 0, 0);
    }
    #pragma omp simd
    for(int i = 0; i < SDFSize[1]; i++){
        yVals[i] = glm::vec3(0, i*interval, 0);
    }
    #pragma omp simd
    for(int i = 0; i < SDFSize[2]; i++){
        zVals[i] = glm::vec3(0, 0, i*interval);
    }

    #pragma omp simd collapse(3)
    for(int i = 0; i < SDFSize[0]; i++){
        for(int j = 0; j < SDFSize[1]; j++){
            for(int k = 0; k < SDFSize[2]; k++){
                XYZ[i*SDFSize[1]*SDFSize[2] + j*SDFSize[2] + k] = xVals[i] + yVals[j] + zVals[k];
            }
        }
    }


    //Initialises SDF by assuming all points are well outside the model
    vector<float> SDFSoFar;
    #pragma omp simd
    for(int i = 0; i < totalSDFSize; i++){
        SDFSoFar[i] = 1e6f;
    }

    int numParts = extrusions.size();

    //Gets SDF of each part
    for(int i = 0; i < numParts; i++){

        //Get SDF of part
        vector<float> partSDF;
        //Finds indices of edge of bounding box of part
        glm::imat2x3 boundingIndices;
        boundingIndices[0] = ceil((boundingBoxes[0][0]-totalBoundingBox[0])/boundSize*glm::vec3(SDFSize - glm::ivec3(1, 1, 1)));
        boundingIndices[1] = floor((boundingBoxes[0][1]-totalBoundingBox[0])/boundSize*glm::vec3(SDFSize - glm::ivec3(1, 1, 1)));

        glm::ivec3 partSDFSize = generatePartSDF(extrusions, profiles[profileIndices[i]], i, parentIndices[i],
            XYZ, SDFSize, 
            boundingIndices, surfMeshRes, partSDF);

        //Find minimum of part SDF and total SDF for points in bounding box
        #pragma omp simd collapse(3)
        for(int x = 0; x < partSDFSize[0]; x++){
            for(int y = 0; y < partSDFSize[1]; y++){
                for(int z = 0; z < partSDFSize[2]; z++){
                    int index = (x + boundingIndices[0][0])*SDFSize[1]*SDFSize[2] + 
                        (y + boundingIndices[0][1])*SDFSize[2] + (z + boundingIndices[0][2]);
                    int partIndex = x*partSDFSize[1]*partSDFSize[2] + y*partSDFSize[2] + z;

                    //Vectorisable min function
                    SDFSoFar[index] = SDFSoFar[index] + (partSDF[partIndex] - SDFSoFar[index]) * (partSDF[partIndex] > SDFSoFar[index]);
                }
            }
        }

    }

    return SDFSize;
}


glm::ivec3 generatePartSDF(const vector<extrusionData>& extrusions, const profile& partProfile, 
    int partIndex, vector<int> parentIndices, const vector<glm::vec3>& meshGrid, glm::ivec3 SDFSize, 
    glm::imat2x3 boundingIndices, double surfMeshRes, vector<float>& SDF){

    //Adds one to end indices so because the boundingIndices are inclusive
    glm::ivec3 endBoundingIndices = boundingIndices[1] + glm::ivec3(1, 1, 1);
    glm::ivec3 partSDFSize = endBoundingIndices - boundingIndices[0];
    int totalPartSDFSize = partSDFSize[0] * partSDFSize[1] * partSDFSize[2];

    //Copies coordinates from within bounding box to local variables
    vector<glm::vec3> XYZ;
    XYZ.resize(partSDFSize[0]);

    #pragma omp simd collapse(3)
    for(int x = 0; x < partSDFSize[0]; x++){
        for(int y = 0; y < partSDFSize[1]; y++){
            for(int z = 0; z < partSDFSize[2]; z++){
                int index = (x + boundingIndices[0][0])*SDFSize[1]*SDFSize[2] + 
                    (y + boundingIndices[0][1])*SDFSize[2] + (z + boundingIndices[0][2]);
                int pIndex = x*partSDFSize[1]*partSDFSize[2] + y*partSDFSize[2] + z;

                XYZ[pIndex] = meshGrid[index];
            }
        }
    }

    //Gets transformations to be applied to coordinates
    int numParents = parentIndices.size();
    vector<glm::vec3> translations;
    translations.resize(numParents + 1);
    vector<glm::quat> rotations;
    rotations.resize(numParents + 1);
    vector<glm::vec3> pivotPoints;
    pivotPoints.resize(numParents + 1);

    //Adds part transformations to list
    parentIndices.insert(parentIndices.begin(), partIndex);
    
    for(int i = 0; i < numParents + 1; i++){
        translations[i] = -1.0*extrusions[parentIndices[numParents - i]].translation;
        rotations[i] = glm::inverse(extrusions[parentIndices[numParents - i]].rotation);
        pivotPoints[i] = extrusions[parentIndices[numParents - i]].pivotPoint;

        //Pre-Adds the pivot point to the translation as the translation from the pivot point
        //to the origin is the next step in the transformation anyway
        translations[i] += pivotPoints[i];
    }


    //Applies transformations to coordinates
    #pragma omp simd collapse(2)
    for(int p = 0; p < numParents + 1; p++){
        for(int i = 0; i < totalPartSDFSize; i++){

            XYZ[i] += translations[i];
            XYZ[i] = rotations[i] * XYZ[i];
            XYZ[i] -= pivotPoints[i];
        }
    }


    //Precalculates tables of values related to extrusion scaling and translation along its length
    
    //Gets required bounds of table
    float minZ = XYZ[0][2];
    float maxZ = XYZ[0][2];
    for(int i = 1; i < totalPartSDFSize; i++){
        minZ += (XYZ[i][2] - minZ) * (XYZ[i][2] < minZ);
        maxZ += (XYZ[i][2] - maxZ) * (XYZ[i][2] > maxZ);
    }

    //Accounts for the fact that the length of the table in the z direction is slightly larger
    //than the bounds of the SDF due to the SDF bounds not dividing exactly by the interval
    int numZ = ceil((maxZ - minZ)*surfMeshRes) + 1;
    float interval = 1/surfMeshRes;
    float endZ = maxZ - fmod(maxZ - minZ, interval) + interval;
    float tableZRange = endZ - minZ;

    
    vector<float> scaleTable;
    scaleTable.resize(numZ);
    vector<float> xPosTable;
    xPosTable.resize(numZ);
    vector<float> yPosTable;
    yPosTable.resize(numZ);

    //Inserts values into vector with the start and end values added on to either side
    vector<double> zSample, scaleVals, xPosVals, yPosVals;
    int numVals = extrusions[partIndex].zSampleVals.size() + 2;
    zSample.resize(numVals); scaleVals.resize(numVals);
    xPosVals.resize(numVals); yPosVals.resize(numVals);
    copy(extrusions[partIndex].zSampleVals.begin(), extrusions[partIndex].zSampleVals.end(), zSample.begin()+1);
    copy(extrusions[partIndex].scaleVals.begin(), extrusions[partIndex].scaleVals.end(), scaleVals.begin()+1);
    copy(extrusions[partIndex].xPosVals.begin(), extrusions[partIndex].xPosVals.end(), xPosVals.begin()+1);
    copy(extrusions[partIndex].yPosVals.begin(), extrusions[partIndex].yPosVals.end(), yPosVals.begin()+1);
    scaleVals[0] = scaleVals[1]; scaleVals[numVals - 1] = scaleVals[numVals - 2];
    xPosVals[0] = xPosVals[1]; xPosVals[numVals - 1] = xPosVals[numVals - 2];
    yPosVals[0] = yPosVals[1]; yPosVals[numVals - 1] = yPosVals[numVals - 2];

    //Accounts for extrusions in opposite direction
    if(zSample[1] > zSample[numVals-2]){
        reverse(zSample.begin(), zSample.end());
        reverse(scaleVals.begin(), scaleVals.end());
        reverse(xPosVals.begin(), xPosVals.end());
        reverse(yPosVals.begin(), yPosVals.end());
    }

    zSample[0] = minZ;
    zSample[numVals-1] = endZ;
    vector<int> segmentNums;
    segmentNums.resize(numZ);
    vector<float> zTable;
    zTable.resize(numZ);

    for(int i = 0; i < numZ; i++){
        zTable[i] = minZ + (float)i * interval;
        
        //Gets location of current z value in list
        segmentNums[i] = lower_bound(zSample.begin()+1, zSample.end(), zTable[i]) - zSample.begin()-1;
    }

    #pragma omp simd
    for(int i = 0; i < numZ; i++){
        //Linearly interpolates between the two points either side of the segment
        float fraction = (zTable[i] - zSample[segmentNums[i]])/(zSample[segmentNums[i] + 1] - zSample[segmentNums[i] + 1]);
        scaleTable[i] = scaleVals[segmentNums[i]] * (1.0f - fraction) + scaleVals[segmentNums[i] + 1] * fraction;
        xPosTable[i] = xPosVals[segmentNums[i]] * (1.0f - fraction) + xPosVals[segmentNums[i] + 1] * fraction;
        yPosTable[i] = yPosVals[segmentNums[i]] * (1.0f - fraction) + yPosVals[segmentNums[i] + 1] * fraction;
    }


    //Scales and translates coordinates along the extrusion length
    #pragma omp simd
    for(int i = 0; i < totalPartSDFSize; i++){
        //Gets segment of current point
        int segmentNum = (int)((XYZ[i][2] - minZ)/tableZRange*(float)(numZ - 1));

        //Gets values of transformation using linear interpolation
        float fraction = (XYZ[i][2] - zTable[segmentNum])/interval;
        float scale = scaleTable[segmentNum] * (1.0f - fraction) + scaleTable[segmentNum + 1] * fraction;
        float xPos = xPosTable[segmentNum] * (1.0f - fraction) + xPosTable[segmentNum + 1] * fraction;
        float yPos = yPosTable[segmentNum] * (1.0f - fraction) + yPosTable[segmentNum + 1] * fraction;

        //Applies transformations
        XYZ[i][0] = (XYZ[i][0] - xPos)/scale;
        XYZ[i][1] = (XYZ[i][1] - yPos)/scale;

    }


    //Precalculates table of values of distances to part
    
    //Gets useful values
    vector<glm::vec2> v = partProfile.vertexCoords;
    int numVerts = v.size();

    //Point outside model
    glm::vec2 pOut = glm::vec2(21000.245, 1500.245);


    vector<float> a1, b1, c1, d2, l2;
    vector<glm::vec2> v2;
    vector<glm::vec2> diff;
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


    //Gets bounds of plane
    float minX = XYZ[0][0];
    float maxX = XYZ[0][0];
    float minY = XYZ[0][1];
    float maxY = XYZ[0][1];
    #pragma omp simd reduction(+:minX) reduction(+:maxX) reduction(+:minY) reduction(+:maxY)
    for(int i = 1; i < totalPartSDFSize; i++){
        minX += (XYZ[i][0] - minX) * (XYZ[i][0] < minX);
        maxX += (XYZ[i][0] - maxX) * (XYZ[i][0] > maxX);
        minY += (XYZ[i][1] - minY) * (XYZ[i][1] < minY);
        maxY += (XYZ[i][1] - maxY) * (XYZ[i][1] > maxY);
    }


    
    //Gets size of table given resolution
    glm::vec2 numXY = glm::vec2(ceil((maxX - minX)*surfMeshRes) + 1, ceil((maxY - minY)*surfMeshRes) + 1);
    glm::vec2 endXY = glm::vec2(maxX - fmod(maxX - minX, interval) + interval, maxY - fmod(maxY - minY, interval) + interval);
    glm::vec2 tableXYRange = endXY - glm::vec2(minX, minY);

    vector<float> SDFTable;
    int tableSize = numXY[0]*numXY[1];
    SDFTable.resize(tableSize);

    vector<float> dist2s;
    vector<float> intersections;
    dist2s.resize(numVerts);
    intersections.resize(numVerts);


    vector<float> xTable;
    vector<float> yTable;
    xTable.resize(numXY[0]);
    yTable.resize(numXY[1]);


    //Gets x and y values
    for(int x = 0; x < numXY[0]; x++){
        xTable[x] = minX + x * interval;
    }
    for(int y = 0; y < numXY[1]; y++){
        yTable[y] = minY + y * interval;
    }


    //Fills table with values
    for(int x = 0; x < numXY[0]; x++){
        for(int y = 0; y < numXY[1]; y++){
            
            //Loops over each edge of the profile and checks the distance
            #pragma omp simd
            for(int e = 0; e < numVerts; e++){
                float xVal = xTable[x];
                float yVal = yTable[y];

                float dotProd = (xVal - v[e][0])*diff[e][0] + (yVal - v[e][1])*diff[e][1];
                
                float t = dotProd/l2[e];
                //Limits t to between 0 and 1
                t -= t * (0 > t);
                t += (1 - t) * (1 < t);

                float projx = v[e][0] + t*diff[e][0];
                float projy = v[e][1] + t*diff[e][1];
                float distx = xVal - projx;
                float disty = yVal - projy;

                //Finds distance squared and sign
                dist2s[e] = distx*distx + disty*disty;

                //Gets intersection with line segment between point and outside point
                /*If d1 and d2 have the same sign, their product is positive and so
                the result of the comparison is 0. This indicates that */
                float d1 = a1[e] * xVal + b1[e] * yVal + c1[e];
                int int1 = (d1*d2[e]) < 0.0f;

                ///Find parameters of implicit equation of line going to outside of %polygon
                float a2 = pOut[1] - yVal;
                float b2 = xVal - pOut[0];
                float c2 = pOut[0]*yVal - xVal*pOut[1];

                float d12 = a2*v[e][0] + b2*v[e][1] + c2;
                float d22 = a2*v2[e][0] + b2*v2[e][1] + c2;

                int int2 = (d12*d22) < 0;

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

            int minDist2 = min(dist2s.begin(), dist2s.end()) - dist2s.begin();

            //Assigns distance with sign
            SDFTable[x*numXY[0] + y] = sqrt(minDist2) * (1 - 2*(totIntersections % 2));
        }
    }

    
    //Finds SDF values for profile
    vector<float> profileSDF;
    profileSDF.resize(totalPartSDFSize);

    #pragma omp simd
    for(int i = 0; i < totalPartSDFSize; i++){
        //Gets location in table
        int segmentNumX = (int)((XYZ[i][0] - minX)/tableXYRange[0] * (float)(numXY[0] - 1));
        int segmentNumY = (int)((XYZ[i][1] - minY)/tableXYRange[1] * (float)(numXY[1] - 1));

        //Linear interpolation
        float Q00 = SDFTable[segmentNumX*numXY[0] + segmentNumY];
        float Q01 = SDFTable[(segmentNumX + 1)*numXY[0] + segmentNumY];
        float Q10 = SDFTable[segmentNumX*numXY[0] + segmentNumY + 1];
        float Q11 = SDFTable[(segmentNumX + 1)*numXY[0] + segmentNumY + 1];
        
        float fractionX = (xTable[segmentNumX] - XYZ[i][0])/interval;
        float fractionY = (yTable[segmentNumY] - XYZ[i][1])/interval;

        float SDFy0 = Q00 * (1 - fractionX) + Q10 * fractionX;
        float SDFy1 = Q01 * (1 - fractionX) + Q11 * fractionX;

        profileSDF[i] = SDFy0 * (1 - fractionY) + SDFy1 * fractionY;
    }

    //Gets SDF of the faces on either end of the extrusion
    
    //Gets z locations of planes
    float beginPlaneZ = extrusions[partIndex].zSampleVals[0];
    float endPlaneZ = extrusions[partIndex].zSampleVals[0];
    for(int i = 1; i < (int)extrusions[partIndex].zSampleVals.size(); i++){
        beginPlaneZ = min(beginPlaneZ, (float)extrusions[partIndex].zSampleVals[i]);
        endPlaneZ = max(endPlaneZ, (float)extrusions[partIndex].zSampleVals[i]);
    }

    //Fills SDF by combining the SDF of the profile along the extrusion with the SDF of the planes on
    //either end of the extrusion
    SDF.resize(totalPartSDFSize);
    #pragma omp simd
    for(int i = 0; i < totalPartSDFSize; i++){
        //Maximum of the profile SDF and the two planes
        float beginFace = beginPlaneZ - XYZ[i][2];
        float endFace = XYZ[i][2] - endPlaneZ;
        float maxFace = endFace + (beginFace - endFace) * (beginFace > endFace);

        //Assigns overall maximum value to the SDF
        SDF[i] = maxFace + (profileSDF[i] - maxFace) * (profileSDF[i] > maxFace);
    }

    return partSDFSize;
}

void applyGaussianBlur(float sigma, int n, vector<float>& SDF, glm::ivec3 SDFSize){
    int r = round(((float)n-1)/2);

    glm::ivec3 shrunkSDFSize = SDFSize - glm::ivec3(2*r, 2*r, 2*r);

    vector<float> kernelOuterRow;
    kernelOuterRow.resize(r*2+1);
    float kernelSum = 0;

    //vector describing edge row of nxnxn kernel
    for(int i = 0; i < 2*r+1; i++){
        kernelOuterRow[i] = exp(-0.5*(2*r^2+(i-r)^2)/sqrt(sigma));
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
    for(int x = 0; x < shrunkSDFSize[0]; x++){
        for(int y = 0; y < SDFSize[1]; y++){
            for(int z = 0; z < SDFSize[2]; z++){
                rowMults[x*SDFSize[1]*SDFSize[2] + y*SDFSize[2] + z] = 0.0f;
            }
        }
    }

    
    //x, y and z indices are with respect to the inner portion of the SDF that is not
    //trimmed during the matrix multiplication process
    #pragma omp simd collapse(4) reduction(+:rowMults)
    for(int x = 0; x < shrunkSDFSize[0]; x++){
        for(int y = 0; y < SDFSize[1]; y++){
            for(int z = 0; z < SDFSize[2]; z++){
                for(int i = 0; i < 2*r+1; i++){
                    rowMults[x*SDFSize[1]*SDFSize[2] + y*SDFSize[2] + z] += 
                        SDF[(x + i)*SDFSize[1]*SDFSize[2] + y*SDFSize[2] + z]*kernelOuterRow[i];
                }
            }
        }
    }

    vector<float> colMults;
    colMults.resize(shrunkSDFSize[0] * shrunkSDFSize[1] * SDFSize[2]);
    #pragma omp simd collapse(3)
    for(int x = 0; x < shrunkSDFSize[0]; x++){
        for(int y = 0; y < shrunkSDFSize[1]; y++){
            for(int z = 0; z < SDFSize[2]; z++){
                colMults[x*shrunkSDFSize[1]*SDFSize[2] + y*SDFSize[2] + z] = 0.0f;
            }
        }
    }

    //multiplies kernel with SDF in col direction
    #pragma omp simd collapse(4) reduction(+:colMults)
    for(int x = 0; x < shrunkSDFSize[0]; x++){
        for(int y = 0; y < shrunkSDFSize[1]; y++){
            for(int z = 0; z < SDFSize[2]; z++){
                for(int i = 0; i < 2*r+1; i++){
                    colMults[x*shrunkSDFSize[1]*SDFSize[2] + y*SDFSize[2] + z] += 
                        rowMults[x*SDFSize[1]*SDFSize[2] + (y + i)*SDFSize[2] + z]*kernelOuterRow[i];
                }
            }
        }
    }


    #pragma omp simd collapse(3)
    for(int x = r; x < shrunkSDFSize[0] + r; x++){
        for(int y = r; y < shrunkSDFSize[1] + r; y++){
            for(int z = r; z < shrunkSDFSize[2] + r; z++){
                SDF[x*SDFSize[1]*SDFSize[2] + y*SDFSize[2] + z] = 0;
            }
        }
    }

    #pragma omp simd collapse(4) reduction(+:SDF)
    for(int x = 0; x < shrunkSDFSize[0]; x++){
        for(int y = 0; y < shrunkSDFSize[1]; y++){
            for(int z = 0; z < shrunkSDFSize[2]; z++){
                for(int i = 0; i < 2*r+1; i++){
                    SDF[(x + r)*SDFSize[1]*SDFSize[2] + (y + r)*SDFSize[2] + z + r] += 
                        rowMults[x*shrunkSDFSize[1]*SDFSize[2] + y*SDFSize[2] + z + i];
                }
            }
        }
    }


}