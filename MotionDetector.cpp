#include "StdAfx.h"
#include "MotionDetector.h"
#include "StereoImage.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

MotionDetector::MotionDetector(void)
{
	this->frame			= new IplImage();
	this->difference	= new IplImage();
	this->greyImage		= new IplImage();
	this->temp			= new IplImage();
	this->saliencyMap   = NULL;
	this->movingAverage = new IplImage();
	
	this->firstFrame	= true;
}

MotionDetector::~MotionDetector(void)
{
	cvReleaseImage( &this->greyImage );
	cvReleaseImage( &this->movingAverage );
	cvReleaseImage( &this->temp );
	cvReleaseImage( &this->difference);
	cvReleaseImage( &this->frame);
	cvReleaseMat( &this->saliencyMap);
}

CvMat* MotionDetector::detectMotion(IplImage* decodedFrame)
{
	cout << "COMPUTING MOTION" << endl;
	
	if(firstFrame==true)
	{
		//intialize all necessary image if the first frame is passed
		this->frame			= cvCloneImage(decodedFrame);
		this->difference	= cvCloneImage(decodedFrame);
		this->temp			= cvCloneImage(decodedFrame);
		this->greyImage		= cvCreateImage( cvGetSize(decodedFrame), IPL_DEPTH_8U, 1);
		this->movingAverage = cvCreateImage( cvGetSize(decodedFrame), IPL_DEPTH_32F, 3);
		this->saliencyMap   = cvCreateMat(movingAverage->height, movingAverage->width, CV_8U);

		cvConvertScale(this->frame, this->movingAverage, 1.0, 0.0);
		this->firstFrame = false;
	}
	else
	{
		//compute the running average for the next frames
		cvCopy(decodedFrame, this->frame);
		cvRunningAvg(this->frame, movingAverage, 0.020, NULL);
	}
	
	//convert the scale of the moving average.
	cvConvertScale(movingAverage, this->temp, 1.0, 0.0);

	//calculate the difference between the actual frame and the moving average
	cvAbsDiff(this->frame, this->temp, this->difference);

	//convert the image to grayscale.
	cvCvtColor(this->difference, greyImage, CV_RGB2GRAY);

	//dilate and erode to get the most salient blobs
	cvDilate(greyImage, greyImage, 0, 18);
	cvErode(greyImage, greyImage, 0, 10);
	
	//convert image to mat for further processing
	cvGetMat(greyImage, this->saliencyMap, 0, 1);
	
   	return this->saliencyMap;
}