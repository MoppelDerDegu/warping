// author : Torben Dittrich
// edited by : Christopher Hipp

#pragma once

#include "Stdafx.h"
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

class ImageSaliencyDetector
{
private: 
		void quantify();	// quantizes the input image
		void measure();		// calculate color distance
		void smooth();		// smoothing of the color space
		void build();		// create saliency map as Mat object
		Mat rawImage;		// original image
		Mat tmpImage;		// color pointer (describes the real color of each pixel)
		Mat resImage;		// color palette
		Mat couImage;		// frequency of each color
		Mat disImage;		// weighted distance
		Mat resultHC;		// result of algorithm
		CvMat saliencyMap;  // to convert resultHC into a CvMat
		vector<vector<pair<float, int>>> collection;	// contains each color distance

		float maxColorDistance;  //stores the maximum color distance in each frame
		int numColors;			 //stores the number of colors in each frame
public:
		CvMat* hContrast(IplImage*);	// constructor (path to file)

		ImageSaliencyDetector(void);
		~ImageSaliencyDetector(void);

		float getMaxColorDistance();
		int getNumColors();
	
};



