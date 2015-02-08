#include "frontalization.h"

#include <opencv2/highgui/highgui.hpp>


int main(int argc, char* argv[])
{

    if(argc != 4){
        cout << "Usage: " << argv[0] << " <queryImage> <facialPointsFile> <3DModelFile>" << endl;
        return -1;
    }

    Mat image = imread(argv[1]);
    Mat frontalImage;
    Mat_<float> cameraMatrix, facialFeaturePoints;
    model3D model;

    readFacialFeaturePointFromYML(string(argv[2]), facialFeaturePoints);
//    readCameraMatrixFromYML(string(argv[3]), cameraMatrix);
    read3DModelFromYML(string(argv[3]), model);

    doCameraCalibration(model.threedee, facialFeaturePoints, model.sizeU, model.outA, cameraMatrix);

    frontalizeWithoutSymmetry(image, cameraMatrix, model.refU, model.sizeU, frontalImage);

    imshow("image", image);
    imshow("frontal", frontalImage);
    waitKey();

    imwrite("image.png", image);
    imwrite("frontal.png", frontalImage);

    return 0;
}

