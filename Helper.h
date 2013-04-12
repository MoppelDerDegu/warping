#pragma once

#include "stdafx.h"

struct Helper
{
	static IplImage* getNthFrame(CvCapture* capture, int n);
	static IplImage MatToIplImage(Mat& m);
	static Mat IplImageToMat(IplImage* im);
	static float getDistance(Vertex v1, Vertex v2);
	static float getAverageSaliency(int sumOfSaliencyValues, int numOfPixel);
	static float normalize(float value); // normalizes the value to [0...1]
};
