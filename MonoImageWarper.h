#pragma once

#include "stdafx.h"
#include "ImageWarper.h"

class MonoImageWarper : public ImageWarper
{
public:
	MonoImageWarper(void);
	~MonoImageWarper(void);

	IplImage* warpImage(IplImage* img, Size &destSize, Mat &saliencyMap);
};

