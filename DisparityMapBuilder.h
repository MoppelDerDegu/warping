// author : Torben Dittrich
// edited by : Christopher Hipp

#pragma once

#include "StereoImage.h"

class DisparityMapBuilder
{
private:
	IplImage* gray_left;
	IplImage* gray_right;
	CvMat* saliencyMap;

	float percentageOfDepth;   // indicator to for depth weight
	void computeDepthPercentage(int);

public:
	DisparityMapBuilder(void);
	DisparityMapBuilder(CvSize);
	~DisparityMapBuilder(void);

	CvMat* buildDisparityMapBM(StereoImage*);
	CvMat* buildDisparityMapSGBM(StereoImage*);
	CvMat* buildDisparityMapGC(StereoImage*);

	float getPercentageOfDepth();
};

