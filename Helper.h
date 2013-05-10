#pragma once

#include "stdafx.h"

struct Helper
{
	static IplImage* getNthFrame(CvCapture* capture, int n);
	static IplImage MatToIplImage(Mat& m);
	static Mat IplImageToMat(IplImage* im);
	static float getDistance(Vertex v1, Vertex v2);
	static float getAverageSaliency(int sumOfSaliencyValues, int numOfPixel);
	static float normalize(float value, float max); // normalizes values to [0...1]
	static string getImageType(int number);
	static Mesh deepCopyMesh(const Mesh &m);
	static double euclideanNorm(const Vertex &v);
	static float round (float f);
	static double round (double d);
	static vector<double> meshToDoubleVec(Mesh &m);
	static void doubleVecToMesh(const vector<double> &x, Mesh &result);
	static void drawMeshOverMat(const Mesh &mesh, Mat &mat);
	static double stringToDouble(const string &s);
};
