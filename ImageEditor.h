// author : Torben Dittrich
// edited by : Christopher Hipp

#pragma once

#include "StereoImage.h"
#include <opencv\highgui.h>

class ImageEditor
{
private:
	IplImage* both_eye_view;   // stores the complete frame
	IplImage* left_eye_view;   // stores the left view
	IplImage* right_eye_view;  // stores the right view
	IplImage* blurred_left;    // stores the blurred left image
	IplImage* blurred_right;   // stores the blurred right image
	IplImage* matchedTemplate; // stores the template to be matched

	


public:
	ImageEditor(void);
	ImageEditor(CvSize, int, int);
	~ImageEditor(void);
	
	StereoImage* split_vertical(StereoImage*);			// splits the both_eye_view of a stereo image vertically
	IplImage* mergeImages(IplImage*, IplImage*);		// merges two images:
														// expects the left eye as the first parameter and the right eye as the second
	IplImage* smoothImage(CvMat*, StereoImage*, int);	//blurs the input image
														//The first parameter is the combined saliency map, the second one is the original stereo image 
														//and for the blurring window it expects the number of colors in the frame

};


