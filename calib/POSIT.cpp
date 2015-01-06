#include "POSIT.h"
#include <iostream>

using namespace std;

POSIT::POSIT() {
}

POSIT::~POSIT() {
	cvReleasePOSITObject(&positObject);
	//	cvReleaseMat(&intrinsics);
}

void POSIT::initialize(double cameraMatrix[3 * 3], const double &aWidth, const double &aHeight,
		const double &nearPlane, const double &farPlane) {
	width = aWidth;
	height = aHeight;

	//Generate four model points
	//The first one must be (0,0,0) so we shift all
	shift[0] = modelPoints[0].x;
	shift[1] = modelPoints[0].y;
	shift[2] = modelPoints[0].z;

	for (unsigned int i = 0; i < modelPoints.size(); i++) {
		modelPoints[i].x -= shift[0];
		modelPoints[i].y -= shift[1];
		modelPoints[i].z -= shift[2];
	}

	//Create the POSIT object with the model points
	positObject = cvCreatePOSITObject(&modelPoints[0], (int) modelPoints.size());

	//	intrinsics = cvCreateMat( 3, 3, CV_32F );
	//	cvSetZero( intrinsics );
	//	initializeIntrinsics( width, height );
	//	createOpenGLProjectionMatrix( width, height, nearPlane, farPlane );
	for (int i = 0; i < 9; i++) {
		this->cameraMatrix[i] = cameraMatrix[i];
	}
	//POSIT receives a single focal length
	//	this->cameraMatrix[0]=	FOCAL_LENGTH;
	//	this->cameraMatrix[1*3+1]=	FOCAL_LENGTH;
	this->cameraMatrix[1 * 3 + 1] = this->cameraMatrix[0];
}

void transpose(float *in, int m, int n, float *out) {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			out[i * m + j] = in[j * n + i];
		}
	}
}

void POSIT::poseEstimation() {
	//	CvMatr32f rotation_matrix = new float[9];
	//	CvVect32f translation_vector = new float[3];
	//set posit termination criteria: 100 max iterations, convergence epsilon 1.0e-5
	CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1.0e-4f);
	//	cvPOSIT( positObject, &srcImagePoints[0], FOCAL_LENGTH, criteria, rotation_matrix, translation_vector );
	cvPOSIT(positObject, &srcImagePoints[0], cameraMatrix[0], criteria, rotation_matrix,
			translation_vector);

	createOpenGLMatrixFrom(rotation_matrix, translation_vector);

	float rotation_matrix_trans[9];
	transpose(rotation_matrix, 3, 3, rotation_matrix_trans);

	// RT= [R' T'];
	float RT[4][4];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			RT[i][j] = 0;
		}
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			RT[i][j] = rotation_matrix_trans[i * 3 + j];
		}
	}
	for (int j = 0; j < 3; j++) {
		RT[3][j] = translation_vector[j];
	}
	RT[3][3] = 1;

	CvMat RTMatrix = cvMat(4, 4, CV_32F, RT);

	// shiftR is the translation matrix, inverting the shift we've done before calling POSIT
	float shiftR[4][4];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			shiftR[i][j] = 0;
		}
	}
	shiftR[0][0] = shiftR[1][1] = shiftR[2][2] = shiftR[3][3] = 1;
	for (int j = 0; j < 3; j++) {
		shiftR[j][3] = -shift[j];
	}
	CvMat shiftRMatrix = cvMat(4, 4, CV_32F, shiftR);

	// result = [R' T']*shiftR'
	float result[16];
	CvMat resultMatrix = cvMat(4, 4, CV_32F, result);
	cvGEMM(&RTMatrix, &shiftRMatrix, 1.0, NULL, 0.0, &resultMatrix, CV_GEMM_A_T);

	// return R and T from result
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rotation_matrix[i * 3 + j] = result[i * 4 + j];
		}
	}
	for (int j = 0; j < 3; j++) {
		translation_vector[j] = result[j * 4 + 3];
	}
	//		printMatrix(RT[0],4,4);
	//		printMatrix(shiftR[0],4,4);
	//		printMatrix(result,4,4);

	//Show the results
#ifdef _DEBUG
	cout << "\n\n-......- POSE ESTIMATED -......-\n";
	cout << "\n-.- MODEL POINTS -.-\n";
	for ( size_t p=0; p<modelPoints.size(); p++ )
	cout << modelPoints[p].x << ", " << modelPoints[p].y << ", " << modelPoints[p].z << "\n";

	cout << "\n-.- IMAGE POINTS -.-\n";
	for ( size_t p=0; p<modelPoints.size(); p++ )
	cout << srcImagePoints[p].x << ", " << srcImagePoints[p].y << " \n";

	cout << "\n-.- REAL POSE\n";
	for ( size_t p=0; p<4; p++ )
	cout << poseReal[p] << " | " << poseReal[p+4] << " | " << poseReal[p+8] << " | " << poseReal[p+12] << "\n";

	cout << "\n-.- ESTIMATED POSE\n";
	for ( size_t p=0; p<4; p++ )
	cout << posePOSIT[p] << " | " << posePOSIT[p+4] << " | " << posePOSIT[p+8] << " | " << posePOSIT[p+12] << "\n";
#endif

	//	delete rotation_matrix;
	//	delete translation_vector;
}

void POSIT::createOpenGLMatrixFrom(const CvMatr32f &rotationMatrix,
		const CvVect32f &translationVector) {
	for (int i = 0; i < 9; i++) {
		rotation_matrix[i] = rotationMatrix[i];
	}
	for (int i = 0; i < 3; i++) {
		translation_vector[i] = translationVector[i];
	}

	//coordinate system returned is relative to the first 3D input point	
	for (int f = 0; f < 3; f++) {
		for (int c = 0; c < 3; c++) {
			posePOSIT[c * 4 + f] = rotationMatrix[f * 3 + c]; //transposed
		}
	}
	posePOSIT[3] = 0.0;
	posePOSIT[7] = 0.0;
	posePOSIT[11] = 0.0;
	posePOSIT[12] = translationVector[0];
	posePOSIT[13] = translationVector[1];
	posePOSIT[14] = translationVector[2];
	posePOSIT[15] = 1.0;
}

void POSIT::projectModelPoints(float *pose, vector<CvPoint2D32f> &projectedPoints) {
	// The origin of the coordinates system is in the centre of the image
	projectedPoints.clear();
	CvMat poseMatrix = cvMat(4, 4, CV_32F, pose);
	for (size_t p = 0; p < modelPoints.size(); p++) {
		float modelPoint[] = { modelPoints[p].x, modelPoints[p].y, modelPoints[p].z, 1.0f };
		CvMat modelPointMatrix = cvMat(4, 1, CV_32F, modelPoint);
		float point3D[4];
		CvMat point3DMatrix = cvMat(4, 1, CV_32F, point3D);
		cvGEMM(&poseMatrix, &modelPointMatrix, 1.0, NULL, 0.0, &point3DMatrix, CV_GEMM_A_T);

		//Project the transformed 3D points
		CvPoint2D32f point2D = cvPoint2D32f(0.0, 0.0);
		if (point3D[2] != 0) {
			point2D.x = cameraMatrix[0 * 3 + 0] * point3D[0] / point3D[2];
			point2D.y = cameraMatrix[1 * 3 + 1] * point3D[1] / point3D[2];
		}
		projectedPoints.push_back(point2D);
	}
}

