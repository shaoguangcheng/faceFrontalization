#ifndef __D_UTIL__
#define __D_UTIL__

#include <iostream>
#include <sstream>
//#include <mex.h>

//#define printf mexPrintf

using namespace std;

//const bool DEBUG = true;
const bool DEBUG = false;

const unsigned int TWO = 2;
const unsigned int TRIPLET = 3;
const unsigned int QUADLET = 4;

//debug output
extern ostringstream sout;

template < class T >
string toString(const T &arg)
{
	ostringstream	out;

	out << arg;

	return (out.str());
}

ostream& tab(ostream& output);

void printMatrix(double *in, int m, int n);
void printMatrix(float *in, int m, int n);

void transposeAndFlipY(double *in, int m, int n, double *out);

void transpose(double *in, int m, int n, double *out);

void transpose3dim(unsigned char *image, int gWidth , int gHeight,unsigned char *imageOutput);
void transpose3dimBGR(unsigned char *image, int gWidth , int gHeight,unsigned char *imageOutput);

void getOpenGLMatrices(double *A, double *R, double *T, int width, int height, double mv[16],
		double projectionMatrix[16]);

void getCameraMatricesFromOpenGL(double *A, double *R, double *T, int width, int height,
		double mv[16], double projectionMatrix[16]);

#endif
