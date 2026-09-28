#include "genPartSDF.h"

// https://github.com/thecasterian/Closest-Distance-Transform/blob/main/source/geo3d.c#L163

using namespace std;

void genPartSDF(extrusion partExtrusion, const vector<double>& xVals, 
    const vector<double>& yVals, const vector<double>& zVals, double bandSize,
    vector<double>& SDF){

    int xSize = xVals.size();
    int ySize = yVals.size();
    int zSize = zVals.size();


    SDF.resize(xSize*ySize*zSize);
    fill(SDF.begin(), SDF.end(), INF);

    //int SDFsize = xSize*ySize*zSize;

    // Fast SDF algorithm
    // https://github.com/thecasterian/Closest-Distance-Transform/blob/main/source/geo3d.c


    //cout << "faces" << endl;
    // Loops over faces
    int numFaces = partExtrusion.faces.size();
    for(int f = 0; f < numFaces; f++){
        //cout << "0" << endl;
        glm::ivec3 facePoints = partExtrusion.faces[f];
        glm::dvec3 faceNormal = partExtrusion.faceNormals[f];
        // Outward extrusions
        glm::dvec3 pts[6];
        for(int i = 0; i < 3; i++) pts[i] = partExtrusion.verts[facePoints[i]]; 
        for(int i = 0; i < 3; i++) pts[i+3] = pts[i] + bandSize*faceNormal;
        faceSDF(pts, 1, xVals, yVals, zVals, SDF);

        //cout << "1" << endl;

        // Inward extrusions
        for(int i = 0; i < 3; i++) pts[i] = partExtrusion.verts[facePoints[(3-i)%3]]; 
        for(int i = 0; i < 3; i++) pts[i+3] = pts[i] - bandSize*faceNormal;
        faceSDF(pts, -1, xVals, yVals, zVals, SDF);

        //cout << "2" << endl;

    }


    //cout << "edges" << endl;
    // Loop over edges
    int numEdges = partExtrusion.edges.size();
    for(int e = 0; e < numEdges; e++){
        glm::ivec2 edgePoints = partExtrusion.edges[e];
        glm::dvec3 pts[6];

        pts[0] = partExtrusion.verts[edgePoints[0]];
        pts[1] = partExtrusion.verts[edgePoints[1]];

        glm::dvec3 edgeVec = glm::normalize(pts[1] - pts[0]);

        glm::dvec3 nLeft = partExtrusion.faceNormals[partExtrusion.edgeAdjFaces[e][0]];
        glm::dvec3 nRight = partExtrusion.faceNormals[partExtrusion.edgeAdjFaces[e][1]];

        double sgn = glm::dot(glm::cross(edgeVec, nLeft), nRight) >= 0 ? 1 : -1;
        double angle = vectorAngle(nLeft, nRight);

        
        if (angle <= M_PI/3) {
            if (sgn == 1) {
                pts[2] = pts[0] + bandSize*nLeft;
                pts[3] = pts[1] + bandSize*nLeft;
                pts[4] = pts[0] + bandSize*nRight;
                pts[5] = pts[1] + bandSize*nRight;
            }
            else {
                pts[2] = pts[0] - bandSize*nRight;
                pts[3] = pts[1] - bandSize*nRight;
                pts[4] = pts[0] - bandSize*nLeft;
                pts[5] = pts[1] - bandSize*nLeft;
            }
            edgeSDF(pts, sgn, xVals, yVals, zVals, SDF);
        }
        else if (angle <= 2*M_PI/3) {
            glm::dvec3 nMid = glm::angleAxis(angle/2.0, edgeVec)*nLeft;

            if (sgn == 1){
                pts[2] = pts[0] + sgn*bandSize*nLeft;
                pts[3] = pts[1] + sgn*bandSize*nLeft;
            }else{
                pts[2] = pts[0] + sgn*bandSize*nRight;
                pts[3] = pts[1] + sgn*bandSize*nRight;
            }
            pts[4] = pts[0] + sgn*bandSize*nMid;
            pts[5] = pts[1] + sgn*bandSize*nMid;
            edgeSDF(pts, sgn, xVals, yVals, zVals, SDF);

            pts[2] = pts[0] + sgn*bandSize*nMid;
            pts[3] = pts[1] + sgn*bandSize*nMid;
            if (sgn == 1){
                pts[4] = pts[0] + sgn*bandSize*nRight;
                pts[5] = pts[1] + sgn*bandSize*nRight;
            }else{
                pts[4] = pts[0] + sgn*bandSize*nLeft;
                pts[5] = pts[1] + sgn*bandSize*nLeft;
            }
            edgeSDF(pts, sgn, xVals, yVals, zVals, SDF);
        }
        else {
            glm::dvec3 n1, n2;
            if (sgn == 1) {
                n1 = glm::angleAxis(angle/3.0, edgeVec)*nLeft;
                n2 = glm::angleAxis(2.0*angle/3.0, edgeVec)*nLeft;
            }else{
                n2 = glm::angleAxis(-angle/3.0, edgeVec)*nLeft;
                n1 = glm::angleAxis(-2.0*angle/3.0, edgeVec)*nLeft;
            }

            if (sgn == 1){
                pts[2] = pts[0] + sgn*bandSize*nLeft;
                pts[3] = pts[1] + sgn*bandSize*nLeft;
            }else{
                pts[2] = pts[0] + sgn*bandSize*nRight;
                pts[3] = pts[1] + sgn*bandSize*nRight;
            }
            pts[4] = pts[0] + sgn*bandSize*n1;
            pts[5] = pts[1] + sgn*bandSize*n1;
            edgeSDF(pts, sgn, xVals, yVals, zVals, SDF);
            
            pts[2] = pts[0] + sgn*bandSize*n1;
            pts[3] = pts[1] + sgn*bandSize*n1;
            pts[4] = pts[0] + sgn*bandSize*n2;
            pts[5] = pts[1] + sgn*bandSize*n2;
            edgeSDF(pts, sgn, xVals, yVals, zVals, SDF);

            pts[2] = pts[0] + sgn*bandSize*n2;
            pts[3] = pts[1] + sgn*bandSize*n2;
            if(sgn == 1){
                pts[4] = pts[0] + sgn*bandSize*nRight;
                pts[5] = pts[1] + sgn*bandSize*nRight;
            }else{
                pts[4] = pts[0] + sgn*bandSize*nLeft;
                pts[5] = pts[1] + sgn*bandSize*nLeft;
            }
            edgeSDF(pts, sgn, xVals, yVals, zVals, SDF);
        }

    }


    //cout << "verts" << endl;
    // Loop over vertices
    int numVerts = partExtrusion.verts.size();
    for(int v = 0; v < numVerts; v++){
        vector<int> adjFaces = partExtrusion.vertAdjFaces[v];
        int numAdjFaces = adjFaces.size();
        vector<int> adjVerts(numAdjFaces);
        double adjAngle = 0;
        bool is_convex = true, is_concave = true;

        glm::dvec3 vert = partExtrusion.verts[v];

        // Get pseudonormal of point
        glm::dvec3 axis(0.0);
        for(int i = 0; i < numAdjFaces; i++){
            glm::ivec3 adjFace = partExtrusion.faces[adjFaces[i]];
            for (int p = 0; p < 3; p++) {
                // Checks if point in face is the current vertex
                if (adjFace[p] == v) {
                    adjAngle = vectorAngle(
                        partExtrusion.verts[adjFace[(p+1)%3]]-vert,
                        partExtrusion.verts[adjFace[(p+2)%3]]-vert
                    );
                    adjVerts[i] = adjFace[(p+2)%3];
                }
            }
            axis += adjAngle*partExtrusion.faceNormals[adjFaces[i]];
        }
        axis = glm::normalize(axis);


        // Calculate cone angle
        double angle = 0;
        for (int i = 0; i < numAdjFaces; i++) {
            glm::dvec3 adjNormal = partExtrusion.faceNormals[adjFaces[i]];
            if (glm::dot(axis, adjNormal) > 0.0) {
                angle = max(angle, vectorAngle(axis, adjNormal));
            }
        }


        // Calculate convexity
        for (int i = 0; i < numAdjFaces; i++) {
            double d = glm::dot( partExtrusion.verts[adjVerts[i]]-vert, axis);
            if (d > 0) {
                is_convex = false;
            }
            if (d < 0) {
                is_concave = false;
            }
        }

        if (!is_concave) {
            vertexSDF(vert, bandSize, axis, angle, 1, xVals, yVals, zVals, SDF);
        }

        if (!is_convex) {
            vertexSDF(vert, bandSize, -1.0*axis, angle, -1, xVals, yVals, zVals, SDF);
        }

    }


    // Floodfill algorithm
    int xySize = xSize*ySize;
    bool fillFinished = false;
    while(!fillFinished){
        fillFinished = true;
        for(int i = xySize; i < xSize*ySize*(zSize-1); i++){
            if(SDF[i] == INF){
                if(SDF[i+1] < 0.0 || SDF[i-1] < 0.0 ||
                    SDF[i+xSize] < 0.0 || SDF[i-xSize] < 0.0 ||
                    SDF[i+xySize] < 0.0 || SDF[i-xySize] < 0.0) {
                    fillFinished = false;
                    SDF[i] = -INF;
                }

            }
        }
    }


}