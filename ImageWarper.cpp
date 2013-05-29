#include "ImageWarper.h"
#include "stdafx.h"
#include "QuadSaliencyManager.h"
#include "Solver.h"
#include "Helper.h"
#include "WarpingMath.h"
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
	Solver solver(oldSize);

	initializeMesh(img);
	
	//vector<pair<float, Quad>> _wfMap = qsm.assignSaliencyValuesToQuads(initialMesh, saliency);
	//Mesh contentAwareMesh = solver.redistributeQuads(initialMesh, _wfMap);
	//FileManager::saveMeshAsText("redistributed_mesh.txt", "D:\\warping\\mesh\\", contentAwareMesh);
	Mesh contentAwareMesh = FileManager::loadMesh("D:\\warping\\mesh\\redistributed_mesh.txt");
	vector<pair<float, Quad>> wfMap = qsm.assignSaliencyValuesToQuads(contentAwareMesh, saliency);

	FileManager::saveMeshAsImage("redistributed_mesh.png", "D:\\warping\\mesh\\", contentAwareMesh, oldSize);
	//FileManager::saveMeshAsText("redistributed_mesh.txt", "D:\\warping\\mesh\\", contentAwareMesh);

	deformedMesh = solver.solveImageProblem(contentAwareMesh,initialMesh, destSize, wfMap);
	linearScaledMesh = solver.getInitialGuess();
	
	//linearly scale the image as starting point
	resize(src, tmp, newSize);
	tmp.convertTo(tmp, CV_32FC3);

	FileManager::saveMeshAsImage("blume_mesh.png", "D:\\warping\\mesh\\", deformedMesh, newSize);
	FileManager::saveMeshAsImage("blume_mesh_initial_guess.png", "D:\\warping\\mesh\\", linearScaledMesh, newSize);
	resize(saliency, saliency, newSize);
	Helper::drawMeshOverMat(deformedMesh, saliency);
	FileManager::saveMat("blume_combined_saliency + mesh.png", "D:\\warping\\mesh\\", saliency);

	// do the warping according to mesh
	warp();

	// convert dest frame back to original type
	dest.convertTo(dest, src.type());

	string filename = "warped_image + mesh.png";
	string dir = "D:\\warping\\result\\";
	FileManager::saveMat("warped_image.png", dir, dest);
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

	int quadSizeX = WarpingMath::round((float) img->width / (float) QUAD_NUMBER_X);
	int quadSizeY = WarpingMath::round((float) img->height / (float) QUAD_NUMBER_Y);

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
	cout << ">> Warp image" << endl;

	// vectors of deformed quad
	Vertex u, v, a, b, p, q;

	// vectors of the linear scaled quad
	Vertex _u, _v, _a, _b, _p, _q;

	for (unsigned int i = 0; i < deformedMesh.quads.size(); i++)
	{
		Quad deformedQuad = deformedMesh.quads.at(i); //current Quad
		Quad linearQuad = linearScaledMesh.quads.at(i); // the corresponding quad in the linear scaled mesh
		Quad deformedQuadRelative = Helper::getRelativeCoordinates(deformedQuad); //relative coordinates of quad
		Quad linearQuadRelative = Helper::getRelativeCoordinates(linearQuad); //relative coordinates of quad

		Mat deformedROI, linearROI;

		u = deformedQuadRelative.v3;
		v = deformedQuadRelative.v2;
		a = deformedQuadRelative.v4 - deformedQuadRelative.v3;
		b = deformedQuadRelative.v1 - deformedQuadRelative.v3;
		p = deformedQuadRelative.v1 - deformedQuadRelative.v2;
		q = deformedQuadRelative.v4 - deformedQuadRelative.v2;

		_u = linearQuadRelative.v3;
		_v = linearQuadRelative.v2;
		_a = linearQuadRelative.v4 - linearQuadRelative.v3;
		_b = linearQuadRelative.v1 - linearQuadRelative.v3;
		_p = linearQuadRelative.v1 - linearQuadRelative.v2;
		_q = linearQuadRelative.v4 - linearQuadRelative.v2;

		//set ROI over deformed quad, i.e. a rectangle that engulfs the quad completely
		Helper::getImageROI(deformedQuad, deformedROI, dest);
		Helper::getImageROI(linearQuad, linearROI, tmp);

		for (int j = 0; j < deformedROI.rows; j++)
		{
			for (int k = 0; k < deformedROI.cols; k++)
			{
				//current pixel
				Vertex x;
				x.x = k;
				x.y = j;

				//compute scalars to determine the position of the current pixel
				double r = (double) (b.y * (x.x - u.x) + b.x * (u.y - x.y)) / (double) (b.y * a.x - b.x * a.y);
				double s = (double) (x.y - r * a.y - u.y) / (double) b.y;

				if (!(r < 0.0 || r > 1.0 || s < 0.0 || s > 1.0))
				{
					//pixel is in the parallelogram defined by vectors a and b

					//corresponding pixel in the linear quad
					Vertex _x;
					
					if (r + s < 1.0)
					{
						//pixel is in the lower left triangle of the quad

						_x.x = (int) (_u.x + r * _a.x + s * _b.x);
						_x.y = (int) (_u.y + r * _a.y + s * _b.y);
					}
					else
					{
						double _r = (double) (q.y * (x.x - v.x) + q.x * (v.y - x.y)) / (double) (q.y * p.x - q.x * p.y);
						double _s = (double) (x.y - _r * p.y - v.y) / (double) q.y;

						if (_r + _s <= 1 && !(_r < 0.0 || _r > 1.0 || _s < 0.0 || _s > 1.0))
						{
							//pixel is in the upper right triangle of the quad

							_x.x = (int) (_v.x + _r * _p.x + _s * _q.x);
							_x.y = (int) (_v.y + _r * _p.y + _s * _q.y);
						}
						else
							continue;
					}

					// interpolate pixels and fill new pixel
					if (interpolation == INTER_LINEAR)
					{
						deformedROI.at<Vec3f> (j, k) [0] = interpolateLinear(_x, 0, linearROI);
						deformedROI.at<Vec3f> (j, k) [1] = interpolateLinear(_x, 1, linearROI);
						deformedROI.at<Vec3f> (j, k) [2] = interpolateLinear(_x, 2, linearROI);
					}
					else if (interpolation == INTER_NEAREST)
					{
						deformedROI.at<Vec3f> (j, k) [0] = interpolateNN(_x, 0, linearROI);
						deformedROI.at<Vec3f> (j, k) [1] = interpolateNN(_x, 1, linearROI);
						deformedROI.at<Vec3f> (j, k) [2] = interpolateNN(_x, 2, linearROI);
					}
					else if (interpolation == INTER_CUBIC)
					{
						deformedROI.at<Vec3f> (j, k) [0] = interpolateCubic(_x, 0, linearROI);
						deformedROI.at<Vec3f> (j, k) [1] = interpolateCubic(_x, 1, linearROI);
						deformedROI.at<Vec3f> (j, k) [2] = interpolateCubic(_x, 2, linearROI);
					}
				}
				else
				{
					double _r = (double) (q.y * (x.x - v.x) + q.x * (v.y - x.y)) / (double) (q.y * p.x - q.x * p.y);
					double _s = (double) (x.y - _r * p.y - v.y) / (double) q.y;

					if (_r + _s <= 1 && !(_r < 0.0 || _r > 1.0 || _s < 0.0 || _s > 1.0))
					{
						//pixel is in the upper right triangle of the quad

						Vertex _x;

						_x.x = (int) (_v.x + _r * _p.x + _s * _q.x);
						_x.y = (int) (_v.y + _r * _p.y + _s * _q.y);
						
						// interpolate pixels and fill new pixel
						if (interpolation == INTER_LINEAR)
						{
							deformedROI.at<Vec3f> (j, k) [0] = interpolateLinear(_x, 0, linearROI);
							deformedROI.at<Vec3f> (j, k) [1] = interpolateLinear(_x, 1, linearROI);
							deformedROI.at<Vec3f> (j, k) [2] = interpolateLinear(_x, 2, linearROI);
						}
						else if (interpolation == INTER_NEAREST)
						{
							deformedROI.at<Vec3f> (j, k) [0] = interpolateNN(_x, 0, linearROI);
							deformedROI.at<Vec3f> (j, k) [1] = interpolateNN(_x, 1, linearROI);
							deformedROI.at<Vec3f> (j, k) [2] = interpolateNN(_x, 2, linearROI);
						}
						else if (interpolation == INTER_CUBIC)
						{
							deformedROI.at<Vec3f> (j, k) [0] = interpolateCubic(_x, 0, linearROI);
							deformedROI.at<Vec3f> (j, k) [1] = interpolateCubic(_x, 1, linearROI);
							deformedROI.at<Vec3f> (j, k) [2] = interpolateCubic(_x, 2, linearROI);
						}
					}
				}
			}
		}	
	}
}

float ImageWarper::interpolateLinear(Vertex &x, int channel, Mat &image)
{
	float n, s, w, e;
	
	if (x.y < image.rows && x.x < image.cols)
	{
		if (x.x - 1 < 0)
			w = image.at<Vec3f> (x.y, x.x) [channel];
		else 
			w = image.at<Vec3f> (x.y, x.x - 1) [channel];

		if (x.x + 1 == image.size().width)
			e = image.at<Vec3f> (x.y, x.x) [channel];
		else
			e = image.at<Vec3f> (x.y, x.x + 1) [channel];

		if (x.y - 1 < 0)
			n = image.at<Vec3f> (x.y, x.x) [channel];
		else
			n = image.at<Vec3f> (x.y - 1, x.x) [channel];

		if (x.y + 1 == image.size().height)
			s = image.at<Vec3f> (x.y, x.x) [channel];
		else
			s = image.at<Vec3f> (x.y + 1, x.x) [channel];

		return (float) ((n + s + w + e) / 4);
	}
	else
		return 0.0;
}

float ImageWarper::interpolateNN(Vertex &x, int channel, Mat &image)
{
	if (x.y < image.rows && x.x < image.cols)
		return image.at<Vec3f> (x.y, x.x) [channel];
	else if (x.y == image.rows && x.x == image.cols)
		return image.at<Vec3f> (x.y - 1, x.x - 1) [channel];
	else if (x.y == image.rows)
		return image.at<Vec3f> (x.y - 1, x.x) [channel];
	else if (x.x == image.cols)
		return image.at<Vec3f> (x.y, x.x - 1) [channel];
	else
		return 5000;
}

float ImageWarper::interpolateCubic(Vertex &x, int channel, Mat &image)
{
	// TODO
	return 0.0;
}
