#pragma once

#include "imagewarper.h"
#include "StereoImage.h"

class StereoImageWarper : public ImageWarper
{
public:
	StereoImageWarper(void);
	~StereoImageWarper(void);

	IplImage* warpImage(StereoImage* img, Size &destSize, Mat &saliencyMap);
};

