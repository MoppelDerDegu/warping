#include "ImageWarper.h"
#include "stdafx.h"


ImageWarper::ImageWarper(void)
{
}

ImageWarper::~ImageWarper(void)
{
}

void ImageWarper::setMesh(Mesh& mesh)
{
	this->mesh = mesh;
}

Mesh ImageWarper::getMesh()
{
	return this->mesh;
}

IplImage* ImageWarper::warpImage(IplImage* img, Size destSize)
{
	//initialisation
	this->src = img;
	this->dest = Mat::zeros(destSize, CV_32FC3);
	initializeMesh(img);

	// TODO warp
}

void ImageWarper::initializeMesh(IplImage* img)
{
	// create initial mesh
	int quadSizeX = (int) img->width / QUAD_NUMBER_X;
	int quadSizeY = (int) img->height / QUAD_NUMBER_Y;

	int x;
	int y;
	for (int i = 0; i < QUAD_NUMBER_TOTAL; i++)
	{
		Quad q;

		x = i / QUAD_NUMBER_X;
		y = i % QUAD_NUMBER_Y;

		q.v1.first = x * quadSizeX;
		q.v1.second = y * quadSizeY;

		q.v2.first = (x + 1) * quadSizeX;
		q.v2.second = y * quadSizeY;

		q.v3.first = x * quadSizeX;
		q.v3.second = (y + 1) * quadSizeY;

		q.v4.first = (x + 1) * quadSizeX;
		q.v4.second = (y + 1) * quadSizeY;

		mesh.quads.push_back(q);

		// TODO Add edges
	}
}
