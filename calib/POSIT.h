/****************************************
 *										*
 *			OPENCV POSIT TUTORIAL		*
 *		Javier Barandiaran Martirena	*
 *			       2006					*
 *										*
 ****************************************/
#pragma once

#include <opencv/cxcore.h>
#include <opencv/cv.h>
//#include <GL/glut.h>
#include <vector>

#define FOCAL_LENGTH 760.0

class POSIT {
public:
	POSIT();
	virtual ~POSIT();

	void initialize(double cameraMatrix[3 * 3], const double &aWidth, const double &aHeight,
			const double &nearPlane, const double &farPlane);

	void poseEstimation();
	void
			createOpenGLMatrixFrom(const CvMatr32f &rotationMatrix,
					const CvVect32f &translationVector);
	void projectModelPoints(float *pose, std::vector<CvPoint2D32f> &projectedPoints);

	double width, height; //Image size
	double cameraMatrix[3 * 3];
	float posePOSIT[16];
	float poseReal[16];
	float projectionMatrix[16];
	float shift[3];

	std::vector<CvPoint3D32f> modelPoints;
	std::vector<CvPoint2D32f> srcImagePoints;
	std::vector<CvPoint2D32f> estimatedImagePoints;

	CvPOSITObject* positObject;

	//mine
	float rotation_matrix[9];
	float translation_vector[3];

};
