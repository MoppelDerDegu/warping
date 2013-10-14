#pragma once

#include "stdafx.h"
#include "ImageWarper.h"

// warps a 2D image
class MonoImageWarper : public ImageWarper
{
public:
	MonoImageWarper(void);
	~MonoImageWarper(void);

	IplImage* warpImage(IplImage* img, Size &destSize, Mat &saliencyMap);
};

