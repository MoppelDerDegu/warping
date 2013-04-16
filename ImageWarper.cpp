#include "ImageWarper.h"
#include "stdafx.h"
#include "QuadSaliencyManager.h"
#include "Solver.h"

ImageWarper::ImageWarper(void)
{
}

ImageWarper::~ImageWarper(void)
{
}

void ImageWarper::setMesh(Mesh &mesh)
{
	this->mesh = mesh;
}

Mesh ImageWarper::getMesh()
{
	return this->mesh;
}

IplImage* ImageWarper::warpImage(IplImage* img, Size &destSize, Mat &saliency)
{
	cout << "\nStart Image Warping" << endl;

	//initialisation
	src = img;
	dest = Mat::zeros(destSize, CV_32FC3);
	QuadSaliencyManager qsm;
	initializeMesh(img);
	vector<pair<float, Quad>> wfMap = qsm.assignSaliencyValuesToQuads(mesh, saliency);

	// TODO warp

	return NULL;
}

/*
	The order of the vertices being pushed to the vector is as follows:

	v1---v2---v7---V10
	|	 |	  |		|
	v3---v4---v8---V11
	|	 |    |		|
	v5---v6---v9---V12
*/
void ImageWarper::initializeMesh(IplImage* img)
{
	cout << ">> Initialize Mesh" << endl;

	int quadSizeX = (int) img->width / QUAD_NUMBER_X;
	int quadSizeY = (int) img->height / QUAD_NUMBER_Y;

	int x, y;

	// traverses image column-wise and creates quads
	for (int i = 0; i < QUAD_NUMBER_TOTAL; i++)
	{
		Quad q;
		Edge e1, e2, e3, e4;

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

		e1.src = q.v1;
		e1.dest = q.v2;
		e2.src = q.v2;
		e2.dest = q.v4;
		e3.src = q.v4;
		e3.dest = q.v3;
		e4.src = q.v3;
		e4.dest = q.v1;

		mesh.quads.push_back(q);

		if (i == 0)
		{
			mesh.vertices.push_back(q.v1);
			mesh.vertices.push_back(q.v2);
			mesh.vertices.push_back(q.v3);
			mesh.vertices.push_back(q.v4);
			mesh.edges.push_back(e1);
			mesh.edges.push_back(e2);
			mesh.edges.push_back(e3);
			mesh.edges.push_back(e4);
		}
		else
		{
			if (x == 0)
			{
				// don't add front edge of a quad since it's already part of the mesh
				mesh.edges.push_back(e2);
				mesh.edges.push_back(e3);
				mesh.edges.push_back(e4);

				// don't add v1 and v2 since they are redundant
				mesh.vertices.push_back(q.v3);
				mesh.vertices.push_back(q.v4);
			}
			else
			{
				if (y == 0)
				{
					// don't add the left-hand edge of a quad since it's redundant
					mesh.edges.push_back(e1);
					mesh.edges.push_back(e2);
					mesh.edges.push_back(e3);

					// don't add v1 and v3 since they are redundant
					mesh.vertices.push_back(q.v2);
					mesh.vertices.push_back(q.v4);
				}
				else
				{
					// don't add top and left-hand edge since they are redundant
					mesh.edges.push_back(e2);
					mesh.edges.push_back(e3);

					// only add bottom right vertex
					mesh.vertices.push_back(q.v4);
				}
			}
		}
	}
}
