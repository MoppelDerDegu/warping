#pragma once

#include "stdafx.h"

struct Helper
{
	static IplImage* getNthFrame(CvCapture* capture, int n);
	static IplImage MatToIplImage(Mat& m);
	static Mat IplImageToMat(IplImage* im);
	static string getImageType(int number);
	static Mesh deepCopyMesh(const Mesh &m);
	static vector<double> meshToDoubleVec(Mesh &m);
	static void doubleVecToMesh(const vector<double> &x, Mesh &result);
	static void drawMeshOverMat(const Mesh &mesh, Mat &mat);
	static double stringToDouble(const string &s);
	static Mat meshAsMat(const Mesh &mesh, const Size &s);
	static void getImageROI(Quad &quad, Mat &roi, Mat &img);
	static Quad getRelativeCoordinates(Quad &quad);

	// normalizes a to 0..1 and then multiplies (i,j) in a with (i, j) in b and writes the product in (i,j) in dest
	static void matXmat(const Mat &a, const Mat &b, Mat &dest);
};
