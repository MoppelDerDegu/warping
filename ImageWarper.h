#pragma once

#include "stdafx.h"

class ImageWarper
{
public:
	ImageWarper(void);
	~ImageWarper(void);
	IplImage* warpImage(IplImage* img, Size &dest, Mat &saliency);
	void setMesh(Mesh &mesh);
	Mesh getMesh();
private:
	Vertex newVEnd;
	Vertex vEnd;
	Size oldSize;
	Size newSize;
	Mat src; //source frame
	Mat dest; //destination frame
	Mat tmp;
	Mesh mesh; //the warping mesh
	void initializeMesh(IplImage* img);
};

