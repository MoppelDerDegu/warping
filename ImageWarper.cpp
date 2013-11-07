#include "ImageWarper.h"
#include "Helper.h"

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
void ImageWarper::warp(Mesh &linearScaledMesh, Mesh &deformedMesh, Mat &linearScaledImage, Mat& dest, int interpolation)
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
		Helper::getImageROI(linearQuad, linearROI, linearScaledImage);

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
					Point2f _x;
					
					if (r + s < 1.0)
					{
						//pixel is in the lower left triangle of the quad

						_x.x = _u.x + r * _a.x + s * _b.x;
						_x.y = _u.y + r * _a.y + s * _b.y;
					}
					else
					{
						double _r = (double) (q.y * (x.x - v.x) + q.x * (v.y - x.y)) / (double) (q.y * p.x - q.x * p.y);
						double _s = (double) (x.y - _r * p.y - v.y) / (double) q.y;

						if (_r + _s <= 1 && !(_r < 0.0 || _r > 1.0 || _s < 0.0 || _s > 1.0))
						{
							//pixel is in the upper right triangle of the quad

							_x.x = _v.x + _r * _p.x + _s * _q.x;
							_x.y = _v.y + _r * _p.y + _s * _q.y;
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

						Point2f _x;

						_x.x = (_v.x + _r * _p.x + _s * _q.x);
						_x.y = (_v.y + _r * _p.y + _s * _q.y);
						
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

float ImageWarper::interpolateLinear(Point2f &x, int channel, Mat &image)
{
	double epsilon = 0.01;
	Vertex v1, v2, v3, v4;
	double w1, w2, w3, w4;
	float c1, c2, c3, c4;

	if (x.y < image.rows && x.x < image.cols)
	{
		// nothing to interpolate
		if (x.x - floor(x.x) < epsilon && x.y - floor(x.y) < epsilon)
			return image.at<Vec3f> (x.y, x.x) [channel];

		// values used for interpolation
		v1.x = floor(x.x);
		v1.y = floor(x.y);

		v2.x = ceil(x.x);
		v2.y = floor(x.y);

		v3.x = floor(x.x);
		v3.y = ceil(x.y);

		v4.x = ceil(x.x);
		v4.y = ceil(x.y);

		// check if interpolation points are inside the ROI
		if (v2.x - 1 < 0 || v2.x == image.cols - 1 || v2.y - 1 < 0 || v2.y == image.rows - 1)
			v2 = v1;

		if (v3.x - 1 < 0 || v3.x == image.cols - 1 || v3.y - 1 < 0 || v3.y == image.rows - 1)
			v3 = v1;

		if (v4.x - 1 < 0 || v4.x == image.cols - 1 || v4.y - 1 < 0 || v4.y == image.rows - 1)
			v4 = v1;

		// calculate weights
		w1 = (1 - (x.x - floor(x.x))) * (1 - (x.y - floor(x.y)));
		w2 = (1 - (x.x - floor(x.x))) * (x.y - floor(x.y));
		w3 = (x.x - floor(x.x)) * (1 - (x.y - floor(x.y)));
		w4 = (x.x - floor(x.x)) * (x.y - floor(x.y));

		c1 = image.at<Vec3f> (v1.y, v1.x) [channel];
		c2 = image.at<Vec3f> (v2.y, v2.x) [channel];
		c3 = image.at<Vec3f> (v3.y, v3.x) [channel];
		c4 = image.at<Vec3f> (v4.y, v4.x) [channel];

		return c1 * w1 + c2 * w2 + c3 * w3 + c4 * w4;
	}
	else
		return 0.0;
}

float ImageWarper::interpolateNN(Point2f &x, int channel, Mat &image)
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

float ImageWarper::interpolateCubic(Point2f &x, int channel, Mat &image)
{
	// TODO
	return 0.0;
}