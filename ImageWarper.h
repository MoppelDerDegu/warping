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
	void initializeMesh(IplImage* img);
	void warp(int interpolation = INTER_LINEAR); //warps the destImage according to deformedMesh
	void warpNN();
	void warpLinear();
	void warpCubic();
};

