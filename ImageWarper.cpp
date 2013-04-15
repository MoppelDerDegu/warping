#include "ImageWarper.h"
#include "stdafx.h"
#include "QuadSaliencyManager.h"


ImageWarper::ImageWarper(void)
{
}

ImageWarper::~ImageWarper(void)
{
}

inline void ImageWarper::setMesh(Mesh &mesh)
{
	this->mesh = mesh;
}

inline Mesh ImageWarper::getMesh()
{
	return this->mesh;
}

IplImage* ImageWarper::warpImage(IplImage* img, Size &destSize, Mat &saliency)
{
	//initialisation
	src = img;
	dest = Mat::zeros(destSize, CV_32FC3);
	QuadSaliencyManager qsm;
	initializeMesh(img);
	qsm.assignSaliencyValuesToQuads(mesh, saliency);

	// TODO warp

	return NULL;
}

void ImageWarper::initializeMesh(IplImage* img)
{
	int quadSizeX = (int) img->width / QUAD_NUMBER_X;
	int quadSizeY = (int) img->height / QUAD_NUMBER_Y;

	// create vertices
	int x, y;
	for (int i = 0; i < QUAD_NUMBER_TOTAL; i++)
	{
		Quad q;

		x = (int) i / QUAD_NUMBER_X;
		y = i % QUAD_NUMBER_Y;

		q.v1.x = x * quadSizeX;
		q.v1.y = y * quadSizeY;

		q.v2.x = (x + 1) * quadSizeX;
		q.v2.y = y * quadSizeY;

		q.v3.x = x * quadSizeX;
		q.v3.y = (y + 1) * quadSizeY;

		q.v4.x = (x + 1) * quadSizeX;
		q.v4.y = (y + 1) * quadSizeY;

		mesh.quads.push_back(q);
		mesh.vertices.push_back(q.v1);
		mesh.vertices.push_back(q.v2);
		mesh.vertices.push_back(q.v3);
		mesh.vertices.push_back(q.v4);
	}
}
