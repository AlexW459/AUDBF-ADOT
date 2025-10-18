#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <cmath>
#include <glm/glm.hpp>

using namespace std;
using namespace glm;


int main(){
    

    //Defines constants
    const double rho = 1.225;
    const double  g = 9.81;
    const double atmoPressure = 101325;

    //Runs openFoam
    //system("./runSim.sh");

    //Get face indices of aircraft
    ifstream boundaryFile;
    boundaryFile.open("constant/polyMesh/boundary");

    //Checks that file opened correctly
    if(!boundaryFile) {cout << "Error opening boundary file" << endl; return 1;}


    //Gets data
    string boundaryFileData;
    std::stringstream buffer;
    buffer << boundaryFile.rdbuf();
    boundaryFileData = buffer.str();

    //Finds the relevant data in the file
    regex nFaceExp("(aircraftModel\\n\\s+\\{[^\\{\\}]+nFaces\\s+)(\\d+)(;\\n\\s+startFace\\s+)(\\d+)", regex::multiline);
    smatch faceInfo;
    regex_search(boundaryFileData, faceInfo, nFaceExp);

    //number of faces in the aircraft mesh
    int nFaces = stoi(faceInfo.str(2));
    //Start face (face indices begin at 0)
    int startFace = stoi(faceInfo.str(4));
    //Closes file once the data has been collected
    boundaryFile.close();



    //Opens geometry files
    ifstream faceFile;
    faceFile.open("constant/polyMesh/faces");
    ifstream pointFile;
    pointFile.open("constant/polyMesh/points");\
    ifstream ownerFile;
    ownerFile.open("constant/polyMesh/owner");

    //Opens field files
    ifstream pressureFile;
    pressureFile.open("0/p");
    ifstream velocityFile;
    velocityFile.open("0/U");


    //Check that files opened correctly
    if(!faceFile) {cout << "Error opening face file" << endl; return 1;}
    if(!pointFile) {cout << "Error opening point file" << endl; return 1;}
    if(!ownerFile) {cout << "Error opening owner file" << endl; return 1;}
    if(!pressureFile) {cout << "Error opening pressure file" << endl; return 1;}
    if(!velocityFile) {cout << "Error opening velocity file" << endl; return 1;}


    bool fReached = false;
    bool poReached = false;
    bool oReached = false;
    bool prReached = false;
    bool vReached = false;

    int nPoints;
    int nCells;

    //Moves each file to the starting point of data
    while (!fReached || !poReached || !oReached || !prReached || !vReached){
        regex exp("\\d+");
        string newLine;


        //Checks whether the current line signifies the start of the data
        if (!fReached ){
            getline(faceFile, newLine);

            if(regex_match(newLine, exp)){
                fReached = true;
                //Move an additional line downards to account or the newline
                getline(faceFile, newLine);
            }
        }

        if (!poReached){
            getline(pointFile, newLine);

            if(regex_match(newLine, exp)){
                poReached = true;
                nPoints = stoi(newLine);
                //Move an additional line downards to account or the newline
                getline(pointFile, newLine);
            }
        }

        if (!oReached ){
            getline(ownerFile, newLine);
            if(regex_match(newLine, exp)){
                oReached = true;
                //Move an additional line downards to account or the newline
                getline(ownerFile, newLine);
            }
        }

        if (!prReached ){
            getline(pressureFile, newLine);

            if(regex_match(newLine, exp)){
                nCells = stoi(newLine);
                prReached = true;
                //Move an additional line downards to account or the newline
                getline(pressureFile, newLine);
            }
        }

        if (!vReached ){
            getline(velocityFile, newLine);
            //Sets flag to true when the 
            if(regex_match(newLine, exp)){
                vReached = true;
                //Move an additional line downards to account or the newline
                getline(velocityFile, newLine);
            }
        }
    }

    regex vecExp("(-?\\d+(?:\\.\\d+)?(?:e\\-?\\d+)?) (-?\\d+(?:\\.\\d+)?(?:e\\-?\\d+)?) (-?\\d+(?:\\.\\d+)?(?:e\\-?\\d+)?)");
    
    //Loads point file into vector
    vec3* pointList = new vec3[nPoints];
    for(int i = 0; i < nPoints; i++){
        //Gets next line of file
        string vecS;
        getline(pointFile, vecS);

        //Finds values
        smatch vecMatch;
        regex_search(vecS, vecMatch, vecExp);

        //Allocates memory for point vector
        pointList[i] = vec3(atof((vecMatch.str(1)).c_str()), atof((vecMatch.str(2)).c_str()), atof((vecMatch.str(3)).c_str()));

    }
    pointFile.clear();
    pointFile.close();


    int* ownerList = new int[nFaces];

    //Moves to the relevant part of the owner file
    for (int i = 0; i < startFace; i++){
        string unused;
        getline(ownerFile, unused);
    }
    //Reads in data from each line of the file
    for(int i = 0; i < nFaces; i++){
        string cellNum;
        getline(ownerFile, cellNum);

        ownerList[i] = stoi(cellNum);
    }
    ownerFile.clear();
    ownerFile.close();


    //Gets pressure and velocity data
    float* pressureList = new float[nCells];
    float* velocity2List = new float[nCells];

    for(int i = 0; i < nCells; i++){
        string pressure;

        getline(pressureFile, pressure);


        pressureList[i] = (float)atof((pressure).c_str());

        string velocity;
        getline(velocityFile, velocity);
        smatch vecMatch;
        regex_search(velocity, vecMatch, vecExp);

        float velocities[3] = {(float)atof(vecMatch.str(1).c_str()), (float)atof(vecMatch.str(2).c_str()), (float)atof(vecMatch.str(3).c_str())};
        
        velocity2List[i] = pow(velocities[0], 2) + pow(velocities[1], 2) + pow(velocities[2], 2);

    }
    pressureFile.clear();
    pressureFile.close();
    velocityFile.clear();
    velocityFile.close();

    regex faceExp("(?:\\(| )(\\d+)");

    //Moves to the start of the relevant faces
    for(int i = 0; i < startFace; i++){
        string faceS;
        getline(faceFile, faceS);
    }

    //TEMPORARY. defines COM
    vec3 COM(-0.448, 0.0151, 0.032);


    //Creates variable to store force
    vec3 netForce;
    vec3 netTorque;

    float totArea = 0;
    //Loops over each face in the mesh
    //for (int f = startFace; f < startFace+nFaces-1; f++){
    for (int f = startFace; f < startFace + nFaces; f++){
        string faceS;
        getline(faceFile, faceS);

        //Finds values
        smatch faceMatch;
        regex_search(faceS, faceMatch, faceExp);

        vector<int> vertexIndices;


        string::const_iterator searchStart( faceS.cbegin() );
        while ( regex_search( searchStart, faceS.cend(), faceMatch, faceExp ) )
        {
            string indexS = faceMatch.str(0);
            //Erases the first character from the string because the regex expression captures the presvious character for some unknown reason
            indexS.erase(0, 1);
            vertexIndices.push_back(atoi(indexS.c_str()));
            
            searchStart = faceMatch.suffix().first;
        }

        vec3 centroid(0, 0, 0);
        vector<vec3> vertexList;
        //Enters points into a vector
        for(int i = 0; i < vertexIndices.size(); i++){
            vertexList.push_back(pointList[vertexIndices[i]]);
            
            centroid += vertexList[i];
        }

        

        //Finds central point and average height
        centroid = centroid /= vertexList.size();

        float height = centroid[2];

        vec3 faceNormal;
        float faceArea;

        //Finds cross product of two sides
        vec3 side1 = vertexList[1] - vertexList[0];
        vec3 side2 = vertexList[2] - vertexList[0];
        
        //Vertices are ordered in a counterclockwise direction facing the exterior of the face
        vec3 normal = cross(side1, side2);
        //All faces are boundary faces, and so the normal always points outside the domain,
        //which in this case is towards the interior of the plane, which is the opposite 
        //to the correct direction
        //normal *= -1;
        faceNormal = normalize(normal);


        //Calculates area of face
        if(vertexList.size() == 3){

            faceArea = 0.5*length(normal);

        }else if(vertexList.size() == 4){
            //Finds cross product of two sides
            vec3 side3 = vertexList[3] - vertexList[0];
            
            vec3 normal2 = cross(side2, side3);

            //Adds areas of 2 triangles together to get area of quadrilateral
            faceArea = 0.5*length(normal) + 0.5*length(normal2);

        }else{

            //Assigns the area of the first triangle between an edge of the polygon and the centroid
            float triArea = 0.5*length(cross(vertexList[vertexList.size()-1]-centroid, vertexList[0]-centroid));
            //Loops over edges of the face
            for(int i = 0; i < vertexList.size()-1; i++){
                //Adds area 
                triArea += 0.5*length(cross(vertexList[i]-centroid, vertexList[i+1]-centroid));
            }

            faceArea = triArea;
        }


        totArea += faceArea;

        //Gets index of cell 
        int cellNum = ownerList[f-startFace];

        //Calculates absolute pressure
        float pressure = /*atmoPressure +*/ rho*(pressureList[cellNum] + 0.5*velocity2List[cellNum] + g*height);

        //Calculates forces on face due to pressure
        vec3 pressureForce = (-1.0f)*faceNormal*pressure*faceArea;
        vec3 pressureTorque = cross(centroid-COM, pressureForce);
        
        //Adds values to find net forces
        netForce += pressureForce;
        netTorque += pressureTorque;

    //P_relative = rho*(p + 0.5*U^2 + g*h) 
    }

    cout << "Net Force: ("<< netForce[0] << ", "  << netForce[1] << ", " << netForce[2] << ")\n";
    cout << "Net Torque: ("<< netTorque[0] << ", "  << netTorque[1] << ", " << netTorque[2] << ")\n";

   // cout << "total area: " << totArea << endl;

    //Frees memory
    delete pointList;
    delete ownerList;
    delete pressureList;
    delete velocity2List;


    //Closes files
    faceFile.clear();
    faceFile.close();


    return 0;

}