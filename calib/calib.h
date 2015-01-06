#ifndef CALIB_H
#define CALIB_H

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <opencv/cv.h>
#include "util/calib_util.h"
#include <vector>
#include <cmath>
#include "POSIT.h"

using namespace std;

const string programName = "calib";

int cvStdErrReport2(int code, const char *func_name, const char *err_msg, const char *file, int line, void*);
double distanceMSE(int & numPoints, CvPoint2D64d *& imagePoints, double *& imgPointsIn);

void _cvRQDecomp3x3(const CvMat *matrixM,
                    CvMat *matrixR,
                    CvMat *matrixQ,
                    CvMat *matrixQx,
                    CvMat *matrixQy,
                    CvMat *matrixQz,
                    CvPoint3D64f *eulerAngles);

void doPOSIT(double cameraMatrix[3 * 3],
             CvPoint3D64d *& objectPoints,
             int & numPoints,
             CvPoint2D64d *& imagePoints,
             int & width,
             int & height,
             double rotMatrs[3 * 3],
             double transVects[3]);

void run(int width, int height, int numPoints, double *imgPointsIn, double *objPointsIn,
        double *AOutput, double *ROutput, double *TOutput, double *Ain, double max_mse,
        bool usePosit, bool onlyExtrinsic, int useExtrinsicGuess, double *Rin, double *Tin);

void shutdown(double *imgPoints, double *objPoints);

#endif // CALIB_H
