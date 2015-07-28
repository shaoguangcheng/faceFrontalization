#include "frontalization.h"

void frontalizeWithoutSymmetry(const Mat &image,
                               const Mat_<float> &cameraMatrix,
                               const Mat_<float> &refU,
                               const Size &refSize,
                               Mat &frontalImage,
                               Mat_<float> &projection_,
                               vector<int> &indexFrontal_)
{
    CV_Assert((image.depth() == CV_8U) && (refU.cols == 3));

    // search the invalid 3d point
    vector<int> bgind;
    const Mat_<float> absRefU = cv::abs(refU);
    int nPoint = refU.rows;
    for(int i = 0; i < nPoint; ++i){
        if(equal(static_cast<float>(cv::sum(absRefU.row(i))[0]), static_cast<float>(0.0))){
            bgind.push_back(i);
        }
    }

    // calculate the projection of refU on query image
    Mat_<float> threeDee(nPoint, 4, 1);
    const float* refUPtr;
    float *threeDeePtr;
    for(int i = 0; i < nPoint; ++i){
        refUPtr =refU.ptr<float>(i);
        threeDeePtr = threeDee.ptr<float>(i);

        threeDeePtr[0] = refUPtr[0];
        threeDeePtr[1] = refUPtr[1];
        threeDeePtr[2] = refUPtr[2];
    }

    Mat_<float> tmpProjection = threeDee * cameraMatrix.t();
    Mat_<float> tmpProjection2( nPoint, 2 );

    set<int> badPoint;
    int queryWidth = image.cols, queryHeight = image.rows;
    const float* tmpProjectionPtr;
    float* tmpProjectionPtr2;
    for(int i = 0; i < nPoint; ++i){
        tmpProjectionPtr = tmpProjection.ptr<float>(i);
        tmpProjectionPtr2 = tmpProjection2.ptr<float>(i);

        tmpProjectionPtr2[0] = tmpProjectionPtr[0]/tmpProjectionPtr[2];
        tmpProjectionPtr2[1] = tmpProjectionPtr[1]/tmpProjectionPtr[2];

        // if the projection point is out of plane, record it
        if(std::min(tmpProjectionPtr2[0], tmpProjectionPtr2[1]) < 1.0 ||
                tmpProjectionPtr2[0] > queryWidth-1 ||
                tmpProjectionPtr2[1] > queryHeight-1)
            badPoint.insert(i);
    }

    int sizeBgind = bgind.size();
    for(int i = 0; i < sizeBgind; ++i)
        badPoint.insert(bgind[i]);

    // exculde invalid points from projection
    Mat_<float> projection(nPoint-int(badPoint.size()), 2);
    float* projectionPtr;
    int  k = 0;
    for(int i = 0; i < nPoint; ++i){
        if(badPoint.find(i) == badPoint.end()){
            tmpProjectionPtr2 = tmpProjection2.ptr<float>(i);
            projectionPtr = projection.ptr<float>(k++);

            projectionPtr[0] = tmpProjectionPtr2[0];
            projectionPtr[1] = tmpProjectionPtr2[1];
        }
    }

    vector<int> indexFrontalTmp(refSize.width*refSize.height);
    vector<int> indexFrontal;
    int sizeIndexFrontalTmp = indexFrontalTmp.size();
    for(int i = 0; i < sizeIndexFrontalTmp; ++i)
        indexFrontalTmp[i] = i;
    for(int i = 0; i < sizeIndexFrontalTmp; ++i){
        if(badPoint.find(i) == badPoint.end())
            indexFrontal.push_back(indexFrontalTmp[i]);
    }

    // calculate eachh pixel value of frontal image
    bilinearInterp(image, projection, indexFrontal, refSize, frontalImage);

    projection_   = projection;
    indexFrontal_ = indexFrontal;

    // rotate the frontal image
//    Mat rotMat = getRotationMatrix2D(Point2f(refSize.width/2.0, refSize.height/2.0), 270, 1.0);
//    warpAffine(frontalImage, frontalImage, rotMat, refSize);
}

/////////////////////////////////////////////////////////////////////////////
void bilinearInterp(const Mat& src, const Mat_<float>& pos, const vector<int>& indexFrontal, const Size dstSize, Mat &dst)
{
    int dstHeight = dstSize.height, dstWidth = dstSize.width;

    CV_Assert(pos.cols == 2 && static_cast<int>(pos.rows) ==static_cast<int>( indexFrontal.size()));

    dst.create(dstHeight, dstWidth, src.type());

    const float* posPtr;
    float dx, dy;
    int x, y, row, col;
    int nChannel = src.channels();
    int sizeIndexFrontal = indexFrontal.size();

    if(nChannel == 1){
        for(int i = 0; i < sizeIndexFrontal; ++i){
                posPtr = pos.ptr<float>(i);

                x = static_cast<int>(posPtr[1]);
                y = static_cast<int>(posPtr[0]);
                dx = posPtr[1] - x;
                dy = posPtr[0] - y;

                row = indexFrontal[i]%dstHeight;
                col = indexFrontal[i]/dstHeight;

                dst.at<uchar>(row, col) = saturate_cast<uchar>((1-dx)*(1-dy)*src.at<uchar>(x, y)+
                                                           (1-dx)*dy*src.at<uchar>(x, y+1)+
                                                           dx*(1-dy)*src.at<uchar>(x+1, y)+
                                                           dx*dy*src.at<uchar>(x+1,y+1));
        }
   }
    else if(nChannel == 3){
        Vec3b tmp1, tmp2, tmp3, tmp4;
        for(int i = 0; i < sizeIndexFrontal; ++i){
                posPtr = pos.ptr<float>(i);

                x = static_cast<int>(posPtr[1]); // here must be careful
                y = static_cast<int>(posPtr[0]);
                dx = posPtr[1] - x;
                dy = posPtr[0] - y;

                tmp1 = src.at<Vec3b>(x, y);
                tmp2 = src.at<Vec3b>(x, y+1);
                tmp3 = src.at<Vec3b>(x+1, y);
                tmp4 = src.at<Vec3b>(x+1, y+1);

                row = indexFrontal[i]%dstHeight;
                col = indexFrontal[i]/dstHeight;

                dst.at<Vec3b>(row, col)[0] = saturate_cast<uchar>((1-dx)*(1-dy)*tmp1[0] +
							      (1-dx)*dy*tmp2[0] + dx*(1-dy)*tmp3[0] + dx*dy*tmp4[0]);
                dst.at<Vec3b>(row, col)[1] = saturate_cast<uchar>((1-dx)*(1-dy)*tmp1[1] +
							      (1-dx)*dy*tmp2[1] + dx*(1-dy)*tmp3[1] + dx*dy*tmp4[1]);
                dst.at<Vec3b>(row, col)[2] = saturate_cast<uchar>((1-dx)*(1-dy)*tmp1[2] +
							      (1-dx)*dy*tmp2[2] + dx*(1-dy)*tmp3[2] + dx*dy*tmp4[2]);		
            }
    }
    else{
        DEBUGMSG("Unsupport channel number");
        return;
    }
}


void doCameraCalibration(const Mat_<float>& points3D,
                         const Mat_<float>& points2D,
                         const Size imgSize,
                         const Mat_<float>& intrinsicMatrix,
                         Mat_<float>& cameraMatrix)
{

    int nPoint = points3D.rows;
    CV_Assert(nPoint == points2D.rows &&
              points3D.cols == 3 &&
              points2D.cols == 2);

    double* objPoints = new double [nPoint*3];
    double* imgPoints = new double [nPoint*2];
    double AIn[9];

    const float* point3DPtr, *point2DPtr, *intrMatPtr;

    for(int i = 0; i < nPoint; ++i){
        point3DPtr = points3D.ptr<float>(i);
        point2DPtr = points2D.ptr<float>(i);

        objPoints[i*3]   = static_cast<double>(point3DPtr[0]);
        objPoints[i*3+1] = static_cast<double>(point3DPtr[1]);
        objPoints[i*3+2] = static_cast<double>(point3DPtr[2]);

        imgPoints[i*2]   = static_cast<double>(point2DPtr[0]);
        imgPoints[i*2+1] = static_cast<double>(point2DPtr[1]);
    }

    for(int i = 0; i < 3; ++i){
        intrMatPtr = intrinsicMatrix.ptr<float>(i);

        AIn[i*3]   = static_cast<double>(intrMatPtr[0]);
        AIn[i*3+1] = static_cast<double>(intrMatPtr[1]);
        AIn[i*3+2] = static_cast<double>(intrMatPtr[2]);
    }

    bool usePOSIT = false;

    double* APtr = new double [9];
    double* RPtr = new double [9];
    double* TPtr = new double [3];

    run(imgSize.width, imgSize.height, nPoint, imgPoints, objPoints, APtr, RPtr, TPtr, AIn, 0, usePOSIT, true, 0, NULL, NULL);

    Mat_<float> ROut(3, 3);
    Mat_<float> TOut(3, 1);

    float *ROutPtr, *TOutPtr;

    for(int i = 0; i < 3; ++i){
        ROutPtr = ROut.ptr<float>(i);
        TOutPtr = TOut.ptr<float>(i);

        ROutPtr[0] = static_cast<float>(RPtr[i*3]);
        ROutPtr[1] = static_cast<float>(RPtr[i*3+1]);
        ROutPtr[2] = static_cast<float>(RPtr[i*3+2]);

        TOutPtr[0] = static_cast<float>(TPtr[i]);
    }

    cameraMatrix.create(3, 4);
    float* cameraMatrixPtr;

    ROut = ROut.t();

    for(int i = 0; i < 3; ++i){
        cameraMatrixPtr = cameraMatrix.ptr<float>(i);
        ROutPtr = ROut.ptr<float>(i);
        TOutPtr = TOut.ptr<float>(i);

        cameraMatrixPtr[0] = ROutPtr[0];
        cameraMatrixPtr[1] = ROutPtr[1];
        cameraMatrixPtr[2] = ROutPtr[2];
        cameraMatrixPtr[3] = TOutPtr[0];
    }


    cameraMatrix = intrinsicMatrix*cameraMatrix;

    delete [] objPoints;
    delete [] imgPoints;
    delete [] APtr;
    delete [] RPtr;
    delete [] TPtr;
}
