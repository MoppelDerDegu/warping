#pragma once

#include "imagewarper.h"
#include "StereoImage.h"

class StereoImageWarper : public ImageWarper
{
public:
	StereoImageWarper(void);
	~StereoImageWarper(void);

	Mat warpImage(StereoImage* img, Size &destSize, Mat &saliencyMap); // optimizes the left and right meshe and warps the stereo image
	Mat warpImage(StereoImage* img, Size &destSize, Mesh &deformedLeft, Mesh &deformedRight, Mesh &linearScaledLeft, Mesh &linearScaledRight); // warps a stereo image accourding to the deformed left and right mesh
};

