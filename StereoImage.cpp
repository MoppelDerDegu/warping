// author : Torben Dittrich
// edited by : Christopher Hipp

#include "StdAfx.h"
#include "StereoImage.h"

#include <opencv/highgui.h>
#include <opencv/cv.h>

StereoImage::StereoImage(void)
{

}

StereoImage::StereoImage(CvSize imgSize, int depth, int channels)
{
	//intialize all necessary images
	this->both_eye  = cvCreateImage(imgSize, depth, channels);
	this->left_eye  = cvCreateImage(cvSize(imgSize.width/2,imgSize.height), depth, channels);
	this->right_eye = cvCreateImage(cvSize(imgSize.width/2,imgSize.height), depth, channels);
}


StereoImage::~StereoImage(void)
{
	//release the memory allocated for the images
	cvReleaseImage(&this->both_eye);
	cvReleaseImage(&this->left_eye);
	cvReleaseImage(&this->right_eye);
}

void StereoImage::setBoth_eye(IplImage* img)
{
	this->both_eye = img;
}

IplImage* StereoImage::getBoth_eye()
{
	return both_eye;
}
void StereoImage::setLeft_eye(IplImage* img)
{
	this->left_eye = img;
}

IplImage* StereoImage::getLeft_eye()
{
	return left_eye;
}

void StereoImage::setRight_eye(IplImage* img)
{
	this->right_eye = img;
}

IplImage* StereoImage::getRight_eye()
{
	return right_eye;
}