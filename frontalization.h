#ifndef FRONTALIZATION_H
#define FRONTALIZATION_H

#include <vector>
#include <set>

#include <opencv2/core/core.hpp>

#include "base.h"
#include "frontalUtil.h"
#include "calib/calib.h"

using namespace std;
using namespace cv;

#define ACC_CONST 800

void frontalizeWithoutSymmetry(const Mat& image,
                               const Mat_<float>& cameraMatrix,
                               const Mat_<float>& refU,
                               const Size &refSize,
                               Mat &frontalImage,
                               Mat_<float> &projection_ = *(new Mat_<float>()),
                               vector<int> &indexFrontal_ = *(new vector<int>()));

void frontalizeWithSoftSymmetry(const Mat& image,
                               const Mat_<float>& cameraMatrix,
                               const Mat_<float>& refU,
                               const Size &refSize,
                               const Mat_<float>& eyeMask,
                               Mat& frontalImage);

///////////////////////////////////////////////////
/**
  * @brief using bi-linear interploation to calculate the \a dst according to \a src and the correspondence position \a pos in src
  * @param pos the corresponding position for each points of dst in src (Nx2)
  * @param dstSize the size of dst (WxH)
  * @note the row of pos and the area of dstSize must be the same; src only has single channel.
  */
void bilinearInterp(const Mat& src,
                    const Mat_<float>& pos,
                    const vector<int> &indexFrontal,
                    const Size dstSize,
                    Mat& dst);

/**
  * @brief calculate the camera's parameter
  *
  */
void doCameraCalibration(const Mat_<float>& Points3D,
                         const Mat_<float>& Points2D,
                         const Size imgSize,
                         const Mat_<float> &intrinsicMatrix, Mat_<float> &cameraMatrix);

#endif // FRONTALIZATION_H
