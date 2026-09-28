#ifndef CUSTOMSDF_H
#define CUSTOMSDF_H

#include <glm/glm.hpp>
#include <vector>
#include <iostream>

#include "ishape.h"

namespace sdfibm{

     // SDF is first along x axis, then along y axis, then along z axis
    inline int meshIndexTo1DIndex(int i, int j, int k, int sizeX, int sizeY) {
        return (k * sizeY + j) * sizeX + i;
    }

    inline double pointPosToIndex(const double minBound, const double maxBound, const int numIndices,
        const double pointPos){
        double boundSize = maxBound - minBound;

        return (pointPos-minBound)/boundSize*static_cast<double>(numIndices-1.0);
    }

class Custom_SDF : public IShape, _shapecreator<Custom_SDF>
{
private:
    vector SDFsize;
    vector upperBound;
    vector lowerBound;
    scalarList SDF;
    scalar interval;

public:
    // const static int shape_id = SHAPE::CIRC;
    Custom_SDF(const dictionary& para)
    {
        // fetch needed property from para
        m_radiusB =  Foam::readScalar(para.lookup("boundingRadius"));
        m_com = para.lookupOrDefault("COM", vector::zero);
        m_volume = M_PI*0.5*0.5;//Foam::readScalar(para.lookup("volume"));
        m_volumeINV = 1.0/m_volume;
        m_moi[0] = 1.0; m_moi[1] = 1.0; m_moi[2] = 1.0;

        // Gets SDF properties and values
        SDFsize = para.lookupOrDefault("SDFsize", vector::zero);
        int totalSDFsize = SDFsize[0]*SDFsize[1]*SDFsize[2];
        lowerBound = para.lookupOrDefault("lowerBound", vector::zero);
        upperBound = para.lookupOrDefault("upperBound", vector::zero);
        SDF = para.lookupOrDefault("SDF", scalarList(totalSDFsize));
        interval = (upperBound[0] - lowerBound[0]) / (SDFsize[0] - 1);

        /*m_volume = M_PI*m_radiusSQR; // volume of your shape CHANGE
        m_volumeINV = 1.0/m_volume;
        scalar tmp = 0.5*m_volume*m_radiusSQR; // moi of your shape CHANGE
        m_moi[0] = tmp; m_moi[4] = tmp; m_moi[8] = tmp; // moi of your shape CHANGE
        m_moiINV = Foam::inv(m_moi);
        m_radiusB = m_radius; // bounding radius of your shape CHANGE*/
    }

    // how to get properties of your shape
    //inline scalar getRadius() const { return m_radius;}
    //inline scalar getVolume() const { return m_volume;}

    // typename and description
    SHAPETYPENAME("Custom_SDF")
    // describe your shape
    virtual std::string description() const override {return "Custom shape r = " + std::to_string(m_radiusB);} // CHANGE

    // implement interface
    virtual inline bool isInside(const vector& p) const override
    {
        // tell if a point is inside or outside of your shape

        return signedDistance(p) < 0.0;
    }

    virtual inline scalar signedDistance(const vector& p) const override
    {

        if(p[0] < lowerBound[0] || p[1] < lowerBound[1] || p[2] < lowerBound[2] ||
            p[0] > upperBound[0] || p[1] > upperBound[1] || p[2] > upperBound[2]) return sdf::filter(20.0);

        // Gets position in index coordinates
        int xIndex = pointPosToIndex(lowerBound[0], upperBound[0], SDFsize[0], p[0]);
        int yIndex = pointPosToIndex(lowerBound[1], upperBound[1], SDFsize[1], p[1]);
        int zIndex = pointPosToIndex(lowerBound[2], upperBound[2], SDFsize[2], p[2]);
        int index000 = meshIndexTo1DIndex(xIndex, yIndex, zIndex, SDFsize[0], SDFsize[1]);

        //if(xIndex != pointPosToIndex(lowerBound[0], upperBound[0], SDFsize[0], p[0])) std::cout << xIndex << std::endl;


        return sdf::filter(SDF[index000]);
        
        /*double xd = (p[0] - (lowerBound[0] + xIndex*interval))/interval;
        double yd = (p[1] - (lowerBound[1] + xIndex*interval))/interval;
        double zd = (p[2] - (lowerBound[2] + xIndex*interval))/interval;

        //std::cout << "SDF at index: " << index000 << " is: " << SDF[index000] << std::endl;

        // Eight surrounding points
        int xySize = SDFsize[0]*SDFsize[1];
        double C000, C100, C010, C001, C110, C101, C011, C111;
        C000 = SDF[index000];
        C100 = SDF[index000 + 1]; // Increase in x direction
        C010 = SDF[index000 + SDFsize[0]]; // Increase in y direction
        C001 = SDF[index000 + xySize]; // Increase in z direction
        C110 = SDF[index000 + 1 + SDFsize[0]];
        C101 = SDF[index000 + 1 + xySize];
        C011 = SDF[index000 + SDFsize[0] + xySize];
        C111 = SDF[index000 + 1 + SDFsize[0] + xySize];      

        // Trilinear interpolation
        double C00, C01, C10, C11;
        double xdI = 1-xd;
        C00 = C000*xdI + C100*xd;
        C01 = C001*xdI + C101*xd;
        C10 = C010*xdI + C110*xd;
        C11 = C011*xdI + C111*xd;

        double C0, C1;
        double ydI = 1-yd;
        C0 = C00*ydI + C10*yd;
        C1 = C01*ydI + C11*yd;

        scalar signedDist = C0*(1-zd) + C1*zd;

        

        return sdf::filter(signedDist);*/
    }
private:
   

};

}
#endif
