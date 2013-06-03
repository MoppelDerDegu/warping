#include "ImageWarper.h"
#include "stdafx.h"
#include "QuadSaliencyManager.h"
#include "Solver.h"
#include "Helper.h"
#include "WarpingMath.h"
#include "FileManager.h"
#include "MeshManager.h"

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
	MeshManager* mm = MeshManager::getInstance();
	
	mm->initializeMesh(initialMesh, oldSize);

	//vector<pair<float, Quad>> _wfMap = qsm.assignSaliencyValuesToQuads(initialMesh, saliency);
	//Mesh contentAwareMesh = solver.redistributeQuads(initialMesh, _wfMap);
	//FileManager::saveMeshAsText("redistributed_mesh 20x20.txt", "D:\\warping\\mesh\\", contentAwareMesh);
	Mesh contentAwareMesh = FileManager::loadMesh("D:\\warping\\mesh\\redistributed_mesh 20x20.txt");
	vector<pair<float, Quad>> wfMap = qsm.assignSaliencyValuesToQuads(initialMesh, saliency);

	FileManager::saveMeshAsImage("redistributed_mesh.png", "D:\\warping\\mesh\\", initialMesh, oldSize);
	//FileManager::saveMeshAsText("redistributed_mesh.txt", "D:\\warping\\mesh\\", contentAwareMesh);

	deformedMesh = solver.solveImageProblem(contentAwareMesh, initialMesh, destSize, wfMap);
	linearScaledMesh = solver.getInitialGuess();
	
	//linearly scale the image as starting point
	resize(src, tmp, newSize);
	tmp.convertTo(tmp, CV_32FC3);

	FileManager::saveMat("linearly scaled image.png", "D:\\warping\\result\\", tmp);
	FileManager::saveMeshAsImage("mesh.png", "D:\\warping\\mesh\\", deformedMesh, newSize);
	FileManager::saveMeshAsImage("mesh_initial_guess.png", "D:\\warping\\mesh\\", linearScaledMesh, newSize);

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
		return 0.0;
}

float ImageWarper::interpolateCubic(Vertex &x, int channel, Mat &image)
{
	// TODO
	return 0.0;
}
