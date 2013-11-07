#pragma once

#include "stdafx.h"

// base class of all Warper classes
class ImageWarper
{
protected:
	Size oldSize;
	Size newSize;

	void warp(Mesh &linearScaledMesh, Mesh &deformedMesh, Mat &linearScaledImage, Mat& dest, int interpolation = INTER_LINEAR); // warps an image with regard to the deformed mesh
	float interpolateLinear(Point2f &x, int channel, Mat &image); // interpolates a pixel linearly
	float interpolateNN(Point2f &x, int channel, Mat &image); // interpolates a pixel with nearest neighbor interpolation
	float interpolateCubic(Point2f &x, int channel, Mat &image); // interpolates a pixel cubicly
};

