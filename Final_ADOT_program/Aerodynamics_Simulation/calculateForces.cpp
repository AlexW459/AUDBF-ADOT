#include "calculateForces.h"

using namespace std;


pair<glm::dvec3, glm::dvec3> calculateForces(string filePath, glm::dvec3 COM, const double g, const double rho){    

    //Defines constants
    /*const double rho = 1.225;
    const double  g = 9.81;
    const double atmoPressure = 101325;*/

    //Get face indices of aircraft
    ifstream boundaryFile;
    string boundaryFileName = "0/polyMesh/boundary";
    boundaryFile.open(filePath + boundaryFileName);

    //Checks that file opened correctly
    if(!boundaryFile) throw std::runtime_error("Could not open file \"" + boundaryFileName + "\" in file \"calculateForces.cpp\"");


    //Gets data
    string boundaryS;
    getline(boundaryFile, boundaryS);
    int i = 0;
    while(boundaryS.find("aircraftModel") == string::npos){
        getline(boundaryFile, boundaryS);
        i++;
        if(i > 100) throw std::runtime_error("Could not find aircraftModel in file \"" + boundaryFileName + "\", in file \"calculateForces.cpp\"");
    }

    getline(boundaryFile, boundaryS);
    getline(boundaryFile, boundaryS);
    getline(boundaryFile, boundaryS);
    getline(boundaryFile, boundaryS);
    

    //Finds the relevant data in the file
    smatch faceInfo;
    regex_search(boundaryS, faceInfo, regex("(\\d+)"));

    cout << boundaryS << endl;

    cout << faceInfo.str(0) << endl;

    int nFaces = stoi(faceInfo.str(0));


    getline(boundaryFile, boundaryS);
    //Start face (face indices begin at 0)
    regex_search(boundaryS, faceInfo, regex("(\\d+)"));

        cout << faceInfo.str(0) << endl;

    int startFace = stoi(faceInfo.str(0));
    //Closes file once the data has been collected
    boundaryFile.close();

    cout << "numFaces: " << nFaces << endl;
    cout << "startFace: " << startFace << endl;



    //Opens geometry files
    ifstream faceFile;
    faceFile.open(filePath + "0/polyMesh/faces");
    ifstream pointFile;
    pointFile.open(filePath + "0/polyMesh/points");\
    ifstream ownerFile;
    ownerFile.open(filePath + "0/polyMesh/owner");

    //Opens field files
    ifstream pressureFile;
    pressureFile.open(filePath + "0/p");
    ifstream velocityFile;
    velocityFile.open(filePath + "0/U");


    //Check that files opened correctly
    if(!faceFile) throw std::runtime_error("Could not open file \"0/polyMesh/faces\" in file \"calculateForces.cpp\"");
    if(!pointFile) throw std::runtime_error("Could not open file \"0/polyMesh/points\" in file \"calculateForces.cpp\"");
    if(!ownerFile) throw std::runtime_error("Could not open file \"0/polyMesh/owner\" in file \"calculateForces.cpp\"");
    if(!pressureFile) throw std::runtime_error("Could not open file \"0/p\" in file \"calculateForces.cpp\"");
    if(!velocityFile) throw std::runtime_error("Could not open file \"0/U\" in file \"calculateForces.cpp\"");


    bool fReached = false;
    bool poReached = false;
    bool oReached = false;
    bool prReached = false;
    bool vReached = false;

    int nPoints = 0;
    int nCells = 0;

    //Moves each stream to the starting point of data in each file
    while (!fReached || !poReached || !oReached || !prReached || !vReached){
        regex exp("\\d+");
        string newLine;

        //Checks whether the current line signifies the start of the data
        if (!fReached ){
            getline(faceFile, newLine);

            if(regex_match(newLine, exp)){
                fReached = true;
                //Move an additional line downards to account for the newline
                getline(faceFile, newLine);
            }
        }

        if (!poReached){
            getline(pointFile, newLine);

            if(regex_match(newLine, exp)){
                poReached = true;
                nPoints = stoi(newLine);
                //Move an additional line downards to account for the newline
                getline(pointFile, newLine);
            }
        }

        if (!oReached ){
            getline(ownerFile, newLine);
            if(regex_match(newLine, exp)){
                oReached = true;
                //Move an additional line downards to account for the newline
                getline(ownerFile, newLine);
            }
        }

        if (!prReached ){
            getline(pressureFile, newLine);

            if(regex_match(newLine, exp)){
                nCells = stoi(newLine);
                prReached = true;
                //Move an additional line downards to account for the newline
                getline(pressureFile, newLine);
            }
        }

        if (!vReached ){
            getline(velocityFile, newLine);
            //Sets flag to true when the 
            if(regex_match(newLine, exp)){
                vReached = true;
                //Move an additional line downards to account for the newline
                getline(velocityFile, newLine);
            }
        }
    }


    regex vecExp("(-?\\d+(?:\\.\\d+)?(?:e\\-?\\d+)?) (-?\\d+(?:\\.\\d+)?(?:e\\-?\\d+)?) (-?\\d+(?:\\.\\d+)?(?:e\\-?\\d+)?)");
    
    //Loads point file into vector
    vector<glm::dvec3> pointList;
    pointList.resize(nPoints);


    for(int i = 0; i < nPoints; i++){
        //Gets next line of file
        string vecS;
        getline(pointFile, vecS);

        //Finds values
        smatch vecMatch;
        regex_search(vecS, vecMatch, vecExp);

        //Assigns value to point
        pointList[i][0] = stod(vecMatch.str(1));
        pointList[i][1] = stod(vecMatch.str(2));
        pointList[i][2] = stod(vecMatch.str(3));

    }
    pointFile.clear();
    pointFile.close();

    vector<int> ownerList;
    ownerList.resize(nFaces);


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
    vector<double> pressureList, velocity2List;
    pressureList.resize(nCells);
    velocity2List.resize(nCells);


    for(int i = 0; i < nCells; i++){
        string pressure;

        getline(pressureFile, pressure);

        pressureList[i] = stod(pressure);

        string velocity;
        getline(velocityFile, velocity);
        smatch vecMatch;
        regex_search(velocity, vecMatch, vecExp);

        double Ux = stod(vecMatch.str(1));
        double Uy = stod(vecMatch.str(2));
        double Uz = stod(vecMatch.str(3));
        
        velocity2List[i] = pow(Ux, 2) + pow(Uy, 2) + pow(Uz, 2);

    }
    pressureFile.clear();
    pressureFile.close();
    velocityFile.clear();
    velocityFile.close();

    regex faceExp("(?:\\(| )(\\d+)");
    regex validFace("\\d+\\((?:\\d+ )+\\d+\\)");

    //Moves to the start of the relevant faces
    for(int i = 0; i < startFace; i++){
        string faceS;
        getline(faceFile, faceS);
    }


    //Temp for bugfixing
    double totArea = 0.0;

    glm::dvec3 netForce(0.0, 0.0, 0.0);
    glm::dvec3 netTorque(0.0, 0.0, 0.0);


    //Loops over each face in the mesh
    //for (int f = startFace; f < startFace+nFaces-1; f++){
    for (int f = startFace; f < startFace + nFaces; f++){
        string faceS;
        getline(faceFile, faceS);

        //Checks to make sure that face is valid. If line in the file is too long,
        //it is stretched over multiple lines. This is so rare that these faces are just ignored
        if(!regex_match(faceS, validFace)) continue;


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
            vertexIndices.push_back(stoi(indexS));
            
            searchStart = faceMatch.suffix().first;
        }



        glm::dvec3 centroid(0.0, 0.0, 0.0);
        vector<glm::dvec3> vertexList;
        int numVerts = vertexIndices.size();
        vertexList.resize(numVerts);
        //Enters points into a vector
        for(int i = 0; i < numVerts; i++){

            vertexList[i] = pointList[vertexIndices[i]];
            centroid += vertexList[i];
        }


        //Finds central point and average height
        centroid = centroid / (double)vertexList.size();
        float height = centroid[2];
        glm::dvec3 faceNormal;


        //Finds cross product of two sides
        glm::dvec3 side1 = vertexList[1] - vertexList[0];
        glm::dvec3 side2 = vertexList[2] - vertexList[0];
        

        //Vertices are ordered in a counterclockwise direction facing the exterior of the face
        glm::dvec3 normal = cross(side1, side2);
        //All faces are boundary faces, and so the normal always points outside the domain,
        //which in this case is towards the interior of the plane, which is the opposite 
        //to the correct direction

        faceNormal = -1.0*normalize(normal);


        //Calculates area of face
        double faceArea;
        if(vertexList.size() == 3){

            faceArea = 0.5*length(normal);

        }else if(vertexList.size() == 4){
            //Finds cross product of two sides
            glm::dvec3 side3 = vertexList[3] - vertexList[0];
            
            glm::dvec3 normal2 = cross(side2, side3);

            //Adds areas of 2 triangles together to get area of quadrilateral
            faceArea = 0.5*glm::length(normal) + 0.5*glm::length(normal2);

        }else{

            //Assigns the area of the first triangle between an edge of the polygon and the centroid
            float triArea = 0.5*length(cross(vertexList[vertexList.size()-1]-centroid, vertexList[0]-centroid));
            //Loops over edges of the face
            for(int i = 0; i < numVerts-1; i++){
                //Adds area 
                triArea += 0.5*length(cross(vertexList[i]-centroid, vertexList[i+1]-centroid));
            }

            faceArea = triArea;
        }


        totArea += faceArea;

        //Gets index of cell 
        int cellNum = ownerList[f-startFace];

        //Calculates absolute pressure
        double pressure = rho*(pressureList[cellNum] + 0.5*velocity2List[cellNum] + g*height);

        //Calculates forces on face due to pressure
        glm::dvec3 pressureForce = (-1.0)*faceNormal*pressure*faceArea;
        glm::dvec3 pressureTorque = cross(centroid-COM, pressureForce);
        
        //Adds values to find net forces
        netForce += pressureForce;
        netTorque += pressureTorque;

    //P_relative = rho*(p + 0.5*U^2 + g*h) 
    }

    //Closes files
    faceFile.clear();
    faceFile.close();


    cout << "Net Force: ("<< netForce[0] << ", "  << netForce[1] << ", " << netForce[2] << ")\n";
    cout << "Net Torque: ("<< netTorque[0] << ", "  << netTorque[1] << ", " << netTorque[2] << ")\n";

    cout << "total area: " << totArea << endl;


    return make_pair(netForce, netTorque);
}