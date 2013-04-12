#include "Helper.h"
#include "stdafx.h"

IplImage* Helper::getNthFrame(CvCapture* capture, int n)
{
	for(int i = 0; i <= n; i++)
	{
		if(cvQueryFrame(capture) == NULL)
			return NULL;
	}

	return cvQueryFrame(capture);
}

IplImage Helper::MatToIplImage(Mat& m)
{
	IplImage img = m;
	return img;
}

Mat Helper::IplImageToMat(IplImage* im)
{
	Mat m(im);
	return m;
}

float Helper::getDistance(Vertex v1, Vertex v2)
{
	return sqrt(static_cast<float>(sqr(v1.first - v2.first) + sqr(v1.second - v2.second)));
}

float Helper::getAverageSaliency(int sumOfSaliencyValues, int numOfPixel)
{
	return ((float) sumOfSaliencyValues) / ((float) numOfPixel);
}

float Helper::normalize(float value)
{
	// TODO

	return 0;
}
