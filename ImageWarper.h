#pragma once

#include "stdafx.h"

class ImageWarper
{
public:
	ImageWarper(void);
	~ImageWarper(void);
	IplImage* warpImage(IplImage* img, Size &dest, Mat &saliency);
	Mesh getDeformedMesh();

private:
	Size oldSize;
	Size newSize;
	Mat src; //source frame
	Mat dest; //destination frame
	Mat tmp; //linear scale
	Mesh initialMesh; //the warping mesh
	Mesh linearScaledMesh;
	Mesh deformedMesh;

	void warp(int interpolation = INTER_NEAREST); //warps the destImage according to deformedMesh
	float interpolateLinear(Vertex &x, int channel, Mat &image);
	float interpolateNN(Vertex &x, int channel, Mat &image);
	float interpolateCubic(Vertex &x, int channel, Mat &image);
};

