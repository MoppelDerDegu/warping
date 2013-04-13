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
	Mat m = im;
	return m;
}

float Helper::getDistance(Vertex v1, Vertex v2)
{
	return sqrt(static_cast<float>(sqr(v1.first - v2.first) + sqr(v1.second - v2.second)));
}

float Helper::getAverageSaliency(int sumOfSaliencyValues, int numOfPixel)
{
	float res = ((float) sumOfSaliencyValues) / ((float) numOfPixel);

	if (res < 0)
		return 0;
	else
		return res;
}

float Helper::normalize(float value, float max)
{
	float res = value / max;
	
	if (res < 0)
		return 0;
	else
		return res;
}

string Helper::getImageType(int number)
{
    // find type
    int imgTypeInt = number%8;
    string imgTypeString;

    switch (imgTypeInt)
    {
        case 0:
            imgTypeString = "8U";
            break;
        case 1:
            imgTypeString = "8S";
            break;
        case 2:
            imgTypeString = "16U";
            break;
        case 3:
            imgTypeString = "16S";
            break;
        case 4:
            imgTypeString = "32S";
            break;
        case 5:
            imgTypeString = "32F";
            break;
        case 6:
            imgTypeString = "64F";
            break;
        default:
            break;
    }

    // find channel
    int channel = (number / 8) + 1;

    stringstream type;
    type << "CV_" << imgTypeString << "C" << channel;

    return type.str();
}
