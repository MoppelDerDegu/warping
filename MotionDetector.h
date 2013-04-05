// author : Torben Dittrich
// edited by : Christopher Hipp

#pragma once

#include "StereoImage.h"

#include <opencv\cv.h>
#include <opencv\highgui.h>

class MotionDetector
{
private: 
	IplImage* frame;			//stores the actual frame
	IplImage* difference;		//stores the difference between consecutive frames
	IplImage* movingAverage;	//stores the moving average 
	IplImage* greyImage;		//stores the gray image of the motion map
	IplImage* temp;				//stores a temporarily needed image
	CvMat* saliencyMap;			//the final motion map

	bool firstFrame;			//checks if the first frame is passed

public:
	MotionDetector(void);       //constructor
	~MotionDetector(void);

	CvMat* detectMotion( IplImage* );

};

