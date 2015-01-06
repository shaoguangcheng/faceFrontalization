#ifndef BASE_H
#define BASE_H

#include <iostream>

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

typedef struct model3D
{
    Mat_<float> refU;              // mapping between reference image and 3d model (Nx3)
    Size sizeU;                    // size of refU, just for easy access
    Mat_<float> refXY;             // detected facial points in reference image
    Mat_<float> threedee;          // matching points of refXY in 3D model
    Mat_<float> outA;              // intrinsic matrix of camera
}model3D,*model3DP;


#ifndef DEBUGMSG
#define DEBUGMSG(msg) cout << "line: " <<__LINE__ \
    << ", file: " << __FILE__ \
    << ", message: " << msg << endl;
#endif


#endif // BASE_H
