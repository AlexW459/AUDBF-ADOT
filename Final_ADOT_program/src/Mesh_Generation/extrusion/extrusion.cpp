#include "extrusion.h"

using namespace std;

extrusion::extrusion(){};

extrusion::extrusion(const profile& partProfile, const extrusionData& extrusionInfo){

    // Vertices are numbered in the order they appear in each profile, grouped by their profile,
    // in the order of the profiles.
    // Faces are numbered in the order they appear in the starting profile, then pairs of faces 
    // are numbered the same way as vertices, with the triangle bordering the earlier profile
    // preceding the other triangle in the pair
    // Edges are numbered in the same order as vertices for the starting profile, and then they
    // alternate, with a set of edges perpendicular to the profile (in order) and then a set of 
    // edges parallel to the profiles (also in vertex order)
    
    int numProfiles = extrusionInfo.zVals.size();
    int profileSize = partProfile.vertexCoords.size();

    int numVerts = numProfiles*profileSize;
    verts.resize(numVerts);
    vertAdjFaces.resize(numVerts);
    // Edges on outside of each profile + edges between each profile 
    int numEdges = profileSize + (numProfiles-1)*2*profileSize;
    edges.resize(numEdges);
    // Initialises edge adjacency to negative one so it is known if they have been filled or not
    edgeAdjFaces.resize(numEdges, glm::ivec2(-1));
    // Faces on beginning and end of part + faces between profiles
    int numStartingFaces = partProfile.triangles.size();
    //cout << "numTriangles: " << numStartingFaces << endl; 
    int numFaces = 2*numStartingFaces + (numProfiles-1)*2*profileSize;
    faces.resize(numFaces);
    faceNormals.resize(numFaces);
    //cout << "numVerts: " << profileSize << endl;

    // Checks direction of extrusion
    vector<glm::dvec2> profilePoints = partProfile.vertexCoords;
    vector<glm::ivec3> profileTriangles = partProfile.triangles;
    vector<char> profileAdjMatrix = partProfile.adjacencyMatrix;
    if(extrusionInfo.zVals[1] - extrusionInfo.zVals[0] < 0.0){
        //Flips order of points
        reverse(profilePoints.begin(), profilePoints.end());
        reverse(profileAdjMatrix.begin(), profileAdjMatrix.end());
        //Flips points in triangles
        for(int i = 0; i < numStartingFaces; i++){
            profileTriangles[i][0] = profileSize - partProfile.triangles[i][0] - 1;
            profileTriangles[i][1] = profileSize - partProfile.triangles[i][2] - 1;
            profileTriangles[i][2] = profileSize - partProfile.triangles[i][1] - 1;
        }
    }


    //Finds positions of profiles and vertices
    for(int p = 0; p < numProfiles; p++){
        double zPos = extrusionInfo.zVals[p];
        glm::dvec2 profileTranslate = extrusionInfo.posVals[p];
        glm::dvec2 profileScale = extrusionInfo.scaleVals[p];

        for(int j = 0; j < profileSize; j++){
            //Shifts and scales points
            glm::dvec3 scaledPoint = glm::dvec3(profilePoints[j] * profileScale + profileTranslate, zPos);
            verts[p*profileSize + j] = scaledPoint;
        }
    }

    // Gets faces

    // Starting and end faces
    for(int t = 0; t < numStartingFaces; t++){

        //Vertices of starting and end faces
        int p1 = profileTriangles[t][0];
        int p2 = profileTriangles[t][1];
        int p3 = profileTriangles[t][2];
        int pe1 = numVerts - profileSize + p1;
        int pe2 = numVerts - profileSize + p2;
        int pe3 = numVerts - profileSize + p3;

        faces[t] = glm::ivec3(p1, p2, p3);
        int endT = numFaces - numStartingFaces + t;
        faces[endT] = glm::ivec3(pe1, pe3, pe2);
    }


    // End edges
    for(int e = 0; e < profileSize; e++){
        // Edges of final profile
        // p1 and p2 are calculated to fit edge numbering convention
        int p1, p2;
        if (e != profileSize-1){
            p1 = (numProfiles-1)*profileSize+e+1;
            p2 = (numProfiles-1)*profileSize+e;
        }else{
            p1 = (numProfiles-1)*profileSize;
            p2 = (numProfiles-1)*profileSize+e;
        }
        int edgeIndex = numEdges - profileSize+e;
        edges[edgeIndex] = glm::ivec2(p1, p2);
    }

    // Faces and edges aside from end of extrusion. Diagonal edges aren't added
    for(int p = 0; p < numProfiles-1; p++){
        for(int i = 0; i < profileSize; i++){
            int faceIndex = numStartingFaces + p*2*profileSize + 2*i;
            // p1->p2->p3 is clockwise looking into extrusion, as is p4->p3->p2
            int p1, p2, p3, p4;
            if (i != profileSize-1){
                p1 = p*profileSize+i+1;
                p2 = p*profileSize+i;
                p3 = p1 + profileSize;
                p4 = p2 + profileSize;
            }else{
                p1 = p*profileSize;
                p2 = p*profileSize+profileSize-1;
                p3 = p1 + profileSize;
                p4 = p2 + profileSize;
            }

            faces[faceIndex] = glm::ivec3(p1, p2, p3);
            faces[faceIndex + 1] = glm::ivec3(p4, p3, p2);

            int edgeIndex = p*2*profileSize + i;
            edges[edgeIndex] = glm::ivec2(p1, p2);
            edges[edgeIndex+profileSize] = glm::ivec2(p1, p3);
        }
    }

    // Finds faces adjacent to each vertex
    for(int f = 0; f < numFaces; f++){
        int p1 = faces[f][0];
        int p2 = faces[f][1];
        int p3 = faces[f][2];

        //Adds faces to lists of vertex adjacent faces if not already present
        if(find(vertAdjFaces[p1].begin(), vertAdjFaces[p1].end(), f) == vertAdjFaces[p1].end()){
            vertAdjFaces[p1].push_back(f);}
        if(find(vertAdjFaces[p2].begin(), vertAdjFaces[p2].end(), f) == vertAdjFaces[p2].end()){
            vertAdjFaces[p2].push_back(f);}
        if(find(vertAdjFaces[p3].begin(), vertAdjFaces[p3].end(), f) == vertAdjFaces[p3].end()){
            vertAdjFaces[p3].push_back(f);}
    }


    // Finds faces adjacent to edges
    for(int e = 0; e < numEdges; e++){
        int p1 = edges[e][0];
        int p2 = edges[e][1];

        //Checks faces adjacent to point 1 to find both adjacent faces
        for(int f = 0; f < (int)vertAdjFaces[p1].size(); f++){
            int faceIndex = vertAdjFaces[p1][f];
            int p1Index = 3;
            int p2Index = 3;
            for(int i = 0; i < 3; i++){
                if(faces[faceIndex][i] == p1) p1Index = i;
                if(faces[faceIndex][i] == p2) p2Index = i;
            }

            // Skips face if second point is not attached
            if(p2Index == 3) continue;

            // Checks for orientation of faces
            if((p1Index == 0 && p2Index == 1) || (p1Index == 1 && p2Index == 2)
                || (p1Index == 2 && p2Index == 0)){
                edgeAdjFaces[e][0] =  faceIndex;
            }else{
                edgeAdjFaces[e][1] =  faceIndex;
            }
        }

        // Checks every face for adjacency
        for(int f = 0; f < numFaces; f++){
            int fp1 = faces[f][0];
            int fp2 = faces[f][1];
            int fp3 = faces[f][2];
            if((p1 == fp1 || p1 == fp2 || p1 == fp3) && (p2 == fp1 || p2 == fp2 || p2 == fp3)){
                if(edgeAdjFaces[e][0] == -1){
                    edgeAdjFaces[e][0] = f;
                }else if (edgeAdjFaces[e][0] != f){
                    edgeAdjFaces[e][1] = f;
                }
            }
        }
    }


    int adjMatSize = numVerts;
    adjMatrix.resize(adjMatSize*adjMatSize, 0);
    
    //Adds profile adjacency matrices along diagonals of adjacency matrix
    for(int p = 0; p < numProfiles; p++){
        int profileIndex = p*profileSize;

        for(int r = 0; r < profileSize; r++){
            for(int c = r; c < profileSize; c ++){

                int ind1 = profileIndex + r;
                int ind2 = profileIndex + c;
                adjMatrix[ind1*adjMatSize + ind2] = profileAdjMatrix[r*profileSize + c];
                adjMatrix[ind2*adjMatSize + ind1] = profileAdjMatrix[r*profileSize + c];
            }
        }        
    }

    //Adds identity matrices to off-diagonals
    for(int i = 0; i < profileSize*(numProfiles-1); i++){
        adjMatrix[i*adjMatSize + i+profileSize] = 1;
        adjMatrix[(i+profileSize)*adjMatSize + i] = 1;
    }

    //Splits rectangles into triangles
    for(int p = 0; p < numProfiles-1; p++){
        for(int r = 0; r < profileSize; r++){
            for(int c = r+1; c < profileSize; c++){
                // Copies upper half of profiles to upper half of upper off-diagonal,
                // and vice-verso for lower off-diagonal
                int ind1 = p*profileSize + r;
                int ind2 = (p + 1)*profileSize + c;
                adjMatrix[ind1*adjMatSize + ind2] = profileAdjMatrix[r*profileSize + c];
                adjMatrix[ind2*adjMatSize + ind1] = profileAdjMatrix[r*profileSize + c];
            }
        }
    }


    //Find number of tetrahedrons from profile
    int numTets = 3*profileTriangles.size()*(numProfiles-1);
    tets.resize(numTets);

    //Find tetrahedrons in adjacency matrix
    int tetraNum = 0;
    //Only check upper right half of matrix to avoid double-counting tetrahedrons
    for(int r = 0; r < adjMatSize; r++){
        for(int c = r; c < adjMatSize; c++){
            //Check if there is an adjacency at the current location, and if so enter loop
            for(int p3 = c+1; p3 < adjMatSize*(int)adjMatrix[r*adjMatSize + c]; p3++){
                //Checks if there are two edges that connect both to eachother
                //and to opposite ends of the first edge that was found
                for(int p4 = p3+1; p4 < adjMatSize*(int)adjMatrix[r*adjMatSize + p3]*adjMatrix[p3*adjMatSize + c]; p4++){
                    tets[tetraNum] = glm::ivec4(r, c, p3, p4);
                    //Only increments the index if a tetrahedron was actually found
                    tetraNum += (int)adjMatrix[r*adjMatSize + p4]*adjMatrix[p4*adjMatSize + c]*adjMatrix[p4*adjMatSize + p3];
                    // Finishes if all tetrahedrons have been found
                    if(tetraNum == numTets){
                        goto tetLoopFinish;
                    }
                }
            }
        }
    }
    tetLoopFinish:

    /*cout << "verts:" << endl;
    for(int i = 0; i < numVerts; i++){
        cout << i << ": " << verts[i][0] << ", " << verts[i][1] << ", " << verts[i][2] << endl;
    }

    cout << "edges:" << endl;
    for(int i = 0; i < numEdges; i++){
        cout << i << ": " << edges[i][0] << ", " << edges[i][1] << endl;
    }*/

    computeNormals();
}


