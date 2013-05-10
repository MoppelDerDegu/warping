#include "ImageWarper.h"
#include "stdafx.h"
#include "QuadSaliencyManager.h"
#include "Solver.h"
#include "Helper.h"
#include "FileManager.h"

ImageWarper::ImageWarper(void)
{
}

ImageWarper::~ImageWarper(void)
{
}

Mesh ImageWarper::getDeformedMesh()
{
	return this->deformedMesh;
}

IplImage* ImageWarper::warpImage(IplImage* img, Size &destSize, Mat &saliency)
{
	cout << "\nStart image warping" << endl;

	//initialisation
	oldSize.height = img->height;
	oldSize.width = img->width;
	newSize = destSize;
	src = img;
	dest = Mat::zeros(destSize, CV_32FC3);
	QuadSaliencyManager qsm;
	initializeMesh(img);
	vector<pair<float, Quad>> wfMap = qsm.assignSaliencyValuesToQuads(initialMesh, saliency);
	
	Solver solver;
	deformedMesh = solver.solveImageProblem(initialMesh, destSize, oldSize, wfMap);
	linearScaledMesh = solver.getInitialGuess();
	
	deformedMesh = FileManager::loadMesh("D:\\warping\\mesh\\mesh.txt");

	//linearly scale the image as starting point
	resize(src, tmp, newSize);
	tmp.convertTo(tmp, CV_32FC3);

	// do the warping according to mesh
	warp();

	// convert dest frame back to original type
	dest.convertTo(dest, src.type());

	string filename = "warped_image + mesh.png";
	string dir = "D:\\warping\\result\\";
	Helper::drawMeshOverMat(deformedMesh, dest);
	FileManager::saveMat(filename, dir, dest);

	return &Helper::MatToIplImage(dest);
}

/*
	The order of the vertices being pushed to the vector is as follows:

	v1---v2---v13--v19
	|	 |	  |		|
	v3---v4---v14--v20
	|	 |    |		|
	v5---v6---v15--v21
	|	 |	  |	    |
	v7---v8---v16--v22
	|	 |	  |		|
	v9---v10--v17--v23
	|	 |	  |	    |
	v11--v12--v18--v24
*/
void ImageWarper::initializeMesh(IplImage* img)
{
	cout << ">> Initialize mesh" << endl;

	int quadSizeX = Helper::round((float) img->width / (float) QUAD_NUMBER_X);
	int quadSizeY = Helper::round((float) img->height / (float) QUAD_NUMBER_Y);

	int x, y;

	// traverses image column-wise and creates quads
	for (int i = 0; i < QUAD_NUMBER_TOTAL; i++)
	{
		Quad q;
		Edge e1, e2, e3, e4;

		x = (int) i / QUAD_NUMBER_X;
		y = i % QUAD_NUMBER_Y;

		if (x < QUAD_NUMBER_X - 1 &&  y < QUAD_NUMBER_Y - 1)
		{
			// inner quads
			q.v1.x = x * quadSizeX;
			q.v1.y = y * quadSizeY;

			q.v2.x = (x + 1) * quadSizeX;
			q.v2.y = y * quadSizeY;

			q.v3.x = x * quadSizeX;
			q.v3.y = (y + 1) * quadSizeY;

			q.v4.x = (x + 1) * quadSizeX;
			q.v4.y = (y + 1) * quadSizeY;
		}
		else
		{
			if (x == QUAD_NUMBER_X - 1 && y != QUAD_NUMBER_Y -1)
			{
				// rightmost quads
				q.v1.x = x * quadSizeX;
				q.v1.y = y * quadSizeY;

				q.v2.x = oldSize.width;
				q.v2.y = y * quadSizeY;

				q.v3.x = x * quadSizeX;
				q.v3.y = (y + 1) * quadSizeY;

				q.v4.x = oldSize.width;
				q.v4.y = (y + 1) * quadSizeY;
			}
			else if (x != QUAD_NUMBER_X - 1 && y == QUAD_NUMBER_Y -1)
			{
				// bottom quads
				q.v1.x = x * quadSizeX;
				q.v1.y = y * quadSizeY;

				q.v2.x = (x + 1) * quadSizeX;
				q.v2.y = y * quadSizeY;

				q.v3.x = x * quadSizeX;
				q.v3.y = oldSize.height;

				q.v4.x = (x + 1) * quadSizeX;
				q.v4.y = oldSize.height;
			}
			else if (x == QUAD_NUMBER_X - 1 && y == QUAD_NUMBER_Y -1)
			{
				// bottom right quad
				q.v1.x = x * quadSizeX;
				q.v1.y = y * quadSizeY;

				q.v2.x = oldSize.width;
				q.v2.y = y * quadSizeY;

				q.v3.x = x * quadSizeX;
				q.v3.y = oldSize.height;

				q.v4.x = oldSize.width;
				q.v4.y = oldSize.height;
			}
		}

		e1.src = q.v1;
		e1.dest = q.v2;
		e2.src = q.v2;
		e2.dest = q.v4;
		e3.src = q.v4;
		e3.dest = q.v3;
		e4.src = q.v3;
		e4.dest = q.v1;

		initialMesh.quads.push_back(q);

		if (i == 0)
		{
			initialMesh.vertices.push_back(q.v1);
			initialMesh.vertices.push_back(q.v2);
			initialMesh.vertices.push_back(q.v3);
			initialMesh.vertices.push_back(q.v4);
			initialMesh.edges.push_back(e1);
			initialMesh.edges.push_back(e2);
			initialMesh.edges.push_back(e3);
			initialMesh.edges.push_back(e4);
		}
		else
		{
			if (x == 0)
			{
				// don't add front edge of a quad since it's already part of the mesh
				initialMesh.edges.push_back(e2);
				initialMesh.edges.push_back(e3);
				initialMesh.edges.push_back(e4);

				// don't add v1 and v2 since they are redundant
				initialMesh.vertices.push_back(q.v3);
				initialMesh.vertices.push_back(q.v4);
			}
			else
			{
				if (y == 0)
				{
					// don't add the left-hand edge of a quad since it's redundant
					initialMesh.edges.push_back(e1);
					initialMesh.edges.push_back(e2);
					initialMesh.edges.push_back(e3);

					// don't add v1 and v3 since they are redundant
					initialMesh.vertices.push_back(q.v2);
					initialMesh.vertices.push_back(q.v4);
				}
				else
				{
					// don't add top and left-hand edge since they are redundant
					initialMesh.edges.push_back(e2);
					initialMesh.edges.push_back(e3);

					// only add bottom right vertex
					initialMesh.vertices.push_back(q.v4);
				}
			}
		}
	}
}

/*
	For convenience the following vertices are treated as 2-dimensional vectors.

	Vectors of a quad for warping:
		p
	<--------v
	^		 |
	|		 | q
  b	|		 | 
	|		 V
	u------->
		a
*/
void ImageWarper::warp(int interpolation)
{
	if (interpolation == INTER_NEAREST)
		warpNN();
	else if (interpolation == INTER_LINEAR)
		warpLinear();
	else if (interpolation == INTER_CUBIC)
		warpCubic();
}

void ImageWarper::warpNN()
{
	cout << ">> Warp image with nearest neighbour interpolation" << endl;
	// TODO
}

void ImageWarper::warpLinear()
{
	cout << ">> Warp image with linear interpolation" << endl;

	// vectors of deformed quad
	Vertex u, v, a, b, p, q;

	// vectors of the linear scaled quad
	Vertex _u, _v, _a, _b, _p, _q;

	for (unsigned int i = 0; i < deformedMesh.quads.size(); i++)
	{
		Quad deformedQuad = deformedMesh.quads.at(i);
		Quad linearQuad = linearScaledMesh.quads.at(i); // the corresponding quad in the linear scaled mesh

		Mat deformedROI, linearROI;

		u = deformedQuad.v3;
		v = deformedQuad.v2;
		a = deformedQuad.v4 - deformedQuad.v3;
		b = deformedQuad.v1 - deformedQuad.v3;
		p = deformedQuad.v1 - deformedQuad.v2;
		q = deformedQuad.v4 - deformedQuad.v2;

		_u = linearQuad.v3;
		_v = linearQuad.v2;
		_a = linearQuad.v4 - linearQuad.v3;
		_b = linearQuad.v1 - linearQuad.v3;
		_p = linearQuad.v1 - linearQuad.v2;
		_q = linearQuad.v4 - linearQuad.v2;

		//set ROI over deformed quad, i.e. a rectangle that engulfs the quad completely
		getImageROI(deformedQuad, deformedROI, dest);
		getImageROI(linearQuad, linearROI, tmp);

		for (int j = 0; j < deformedROI.rows; j++)
		{
			for (int k = 0; k < deformedROI.cols; k++)
			{
				//current pixel
				Vertex x;
				x.x = k;
				x.y = j;
				double r = (b.y * (x.x - u.x) + b.x * (u.y - x.y)) / (b.y * a.x - b.x * a.y);
				double s = (x.y - r * a.y - u.y) / b.y;

				if (!(r < 0.0 || r > 1.0 || s < 0.0 || s > 1.0))
				{
					//pixel inside quad

					//corresponding pixel in the linear quad
					Vertex _x;
					
					if (r + s < 1.0)
					{
						_x.x = (int) (_u.x + r * _a.x + s * _b.x);
						_x.y = (int) (_u.y + r * _a.y + s * _b.y);
					}
					else
					{
						_x.x = (int) (_v.x + (1.0 - r) * _p.x + (1.0 - s) * _q.x);
						_x.y = (int) (_v.y + (1.0 - r) * _p.y + (1.0 - s) * _q.y);
					}

					try {
						// interpolate pixels and fill new pixel
						deformedROI.at<Vec3f> (j, k) [0] = interpolateLinear(_x, 0, linearROI);
						deformedROI.at<Vec3f> (j, k) [1] = interpolateLinear(_x, 1, linearROI);
						deformedROI.at<Vec3f> (j, K) [2] = interpolateLinear(_x, 2, linearROI);
					}
					catch(...)
					{
						cout << "i: " << i << " j: " << j << " k: " << k;
						return;
					}
				}
				else
				{
				}
			}
		}
	}
}

void ImageWarper::warpCubic()
{
	cout << ">> Warp image with cubic interpolation" << endl;

	// TODO
}

void ImageWarper::getImageROI(Quad &quad, Mat &roi, Mat &img)
{
	int roiX, roiY, roiWidth, roiHeight;
	Vertex topleft, topright, bottomleft;

	if (quad.v1.y > quad.v2.y)
	{
		roiY = quad.v2.y;
		topright.y = quad.v2.y;
	}
	else
	{
		roiY = quad.v1.y;
		topright.y = quad.v1.y;
	}

	if (quad.v1.x > quad.v3.x)
	{
		roiX = quad.v3.x;
		bottomleft.x = quad.v3.x;
	}
	else
	{
		roiX = quad.v1.x;
		bottomleft.x = quad.v1.x;
	}

	if (quad.v2.x < quad.v4.x)
		topright.x = quad.v4.x;
	else
		topright.x = quad.v2.x;

	if (quad.v3.y < quad.v4.y)
		bottomleft.y = quad.v4.y;
	else
		bottomleft.y = quad.v3.y;

	topleft.y = roiY;
	topleft.x = roiX;

	roiWidth = (int) Helper::getDistance(topleft, topright);
	roiHeight = (int) Helper::getDistance(topleft, bottomleft);

	roi = img(Rect(roiX, roiY, roiWidth, roiHeight));
}

float ImageWarper::interpolateLinear(Vertex &x, int channel, Mat &image)
{
	float n, s, w, e;
	
	if (x.x - 1 < 0)
		w = image.at<Vec3f> (x.x, x.y) [channel];
	else 
		w = image.at<Vec3f> (x.x - 1, x.y) [channel];

	if (x.x + 1 == image.cols)
		e = image.at<Vec3f> (x.x, x.y) [channel];
	else
		e = image.at<Vec3f> (x.x + 1, x.y) [channel];

	if (x.y - 1 < 0)
		n = image.at<Vec3f> (x.x, x.y) [channel];
	else
		n = image.at<Vec3f> (x.x, x.y - 1) [channel];

	if (x.y + 1 == image.rows)
		s = image.at<Vec3f> (x.x, x.y) [channel];
	else
		s = image.at<Vec3f> (x.x, x.y + 1) [channel];

	return (float) ((n + s + w + e) / 4);
}

float ImageWarper::interpolateNN(Vertex &x, int channel, Mat &image)
{
	// TODO
	return 0.0;
}

float ImageWarper::interpolateCubic(Vertex &x, int channel, Mat &image)
{
	// TODO
	return 0.0;
}