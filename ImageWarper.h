#pragma once

#include "stdafx.h"

class ImageWarper
{
protected:
	Size oldSize;
	Size newSize;

	void warp(Mesh &linearScaledMesh, Mesh &deformedMesh, Mat &linearScaledImage, Mat& dest, int interpolation = INTER_NEAREST);
	float interpolateLinear(Vertex &x, int channel, Mat &image);
	float interpolateNN(Vertex &x, int channel, Mat &image);
	float interpolateCubic(Vertex &x, int channel, Mat &image);
};

