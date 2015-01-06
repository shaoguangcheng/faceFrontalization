#ifndef FRONTAL_UTIL_H
#define FRONTAL_UTIL_H

#include <string>
#include <fstream>

#include "base.h"

void read3DModelFromYML(const std::string & fileName,
                        model3D& model);

//just for test image
void readFacialFeaturePointFromYML(const std::string& fileName,
                                   Mat_<float>& facialFeaturePoint);
void readCameraMatrixFromYML(const std::string& fileName,
                             Mat_<float>& cameraMatrix);

void readMatFromFile(const string& fileName, Mat_<float>& m);


template <typename T>
bool equal(const T& x, const T& y)
{
    if(x-y < 1e-5 && x-y > 1e-5)
        return true;
    else
        return false;
}

#endif // FRONTAL_UTIL_H
