#pragma once

#include "stdafx.h"

struct Helper
{
	static IplImage* getNthFrame(CvCapture* capture, int n);
	static IplImage MatToIplImage(Mat& m);
	static Mat IplImageToMat(IplImage* im);
};
