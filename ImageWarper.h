#pragma once

#include "stdafx.h"

class ImageWarper
{
protected:
	Size oldSize;
	Size newSize;

	void warp(Mesh &linearScaledMesh, Mesh &deformedMesh, Mat &linearScaledImage, Mat& dest, int interpolation = INTER_LINEAR);
	float interpolateLinear(Vertex &x, int channel, Mat &image);
	float interpolateNN(Vertex &x, int channel, Mat &image);
	float interpolateCubic(Vertex &x, int channel, Mat &image);
};

