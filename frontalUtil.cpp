#include "frontalUtil.h"

void read3DModelFromYML(const std::string & fileName, model3D& model)
{
    FileStorage fs(fileName,FileStorage::READ);

    if(!fs.isOpened()){
        string msg = "open "  + fileName + " error";
        DEBUGMSG(msg);
        return;
    }

    // read refU from YML
    string refUDataFile = static_cast<string>(fs["refUFile"]);
    int refURows = static_cast<int>(fs["refURows"]);
    int refUCols = static_cast<int>(fs["refUCols"]);

    model.refU.create(refURows, refUCols);
    readMatFromFile(refUDataFile, model.refU);

    fs["refXY"]    >> model.refXY;
    fs["threedee"] >> model.threedee;
    fs["outA"]     >> model.outA;
    model.sizeU.width = int(fs["width"]);
    model.sizeU.height = int(fs["height"]) ;

    fs.release();
}

void readFacialFeaturePointFromYML(const std::string& fileName, Mat_<float>& facialFeaturePoint)
{
    FileStorage fs(fileName,FileStorage::READ);

    if(!fs.isOpened()){
        string msg = "open "  + fileName + " error";
        DEBUGMSG(msg);
        return;
    }

    fs["facialFeaturePoint"] >> facialFeaturePoint;

    fs.release();
}

void readCameraMatrixFromYML(const std::string& fileName, Mat_<float>& cameraMatrix)
{
    FileStorage fs(fileName,FileStorage::READ);

    if(!fs.isOpened()){
        string msg = "open "  + fileName + " error";
        DEBUGMSG(msg);
        return;
    }

    fs["extrinsicMatrix"] >> cameraMatrix;

    fs.release();
}


void readMatFromFile(const string& fileName, Mat_<float>& m)
{
    CV_Assert(m.channels() == 1);

    if(NULL == m.data){
        DEBUGMSG("Mat can not be alloced memory");
        return;
    }

    FILE* fp = fopen(fileName.c_str(), "r");
    if(NULL == fp){
        string msg = "open " + fileName + " error";
        DEBUGMSG(msg);
        return;
    }

    int rows = m.rows, cols = m.cols;
    float* mPtr;
    for(int i = 0; i < rows; ++i){
        mPtr = m.ptr<float>(i);
        for(int j = 0; j < cols; ++j){
            fscanf(fp, "%f,", mPtr+j);
        }
    }

    fclose(fp);
}
