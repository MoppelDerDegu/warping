// author : Torben Dittrich
// edited by : Christopher Hipp

#pragma once

#include <opencv\highgui.h>

class StereoImage
{
	private: 
		IplImage* both_eye;  // stores the complete frame
		IplImage* left_eye;  // stores the left eye view
		IplImage* right_eye; // stores the right eye view

    public:
		StereoImage(void); //constructor
		StereoImage(CvSize, int, int); //constructor to initialize all necessary images
									   //needs the size, depth and channels of the IplImages
		//destructor
		~StereoImage(void);

		//getter and setter
		void setBoth_eye( IplImage* );
		IplImage* getBoth_eye( );
		void setLeft_eye( IplImage* );
		IplImage* getLeft_eye();
		void setRight_eye( IplImage* );
		IplImage* getRight_eye();
};

