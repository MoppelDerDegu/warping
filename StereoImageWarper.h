#pragma once

#include "imagewarper.h"
#include "StereoImage.h"

class StereoImageWarper : public ImageWarper
{
public:
	StereoImageWarper(void);
	~StereoImageWarper(void);

	IplImage* warpImage(StereoImage* img, Size &destSize, Mat &saliencyMap);
	IplImage* warpImage(StereoImage* img, Size &destSize, Mesh &deformedLeft, Mesh &deformedRight, Mesh &linearScaledLeft, Mesh &linearScaledRight);
private:
	IplImage* output;
};

