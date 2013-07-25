#include "WarpingMath.h"

float WarpingMath::getDistance(Vertex v1, Vertex v2)
{
	return sqrt(static_cast<float>(sqr(v1.x - v2.x) + sqr(v1.y - v2.y)));
}

float WarpingMath::getAverageSaliency(int sumOfSaliencyValues, int numOfPixel)
{
	return ((float) sumOfSaliencyValues) / ((float) numOfPixel);
}

float WarpingMath::normalize(float value, float max)
{
	return value / max;
}

double WarpingMath::euclideanNorm(const Vertex &v)
{
	return sqrt((double) (sqr(v.x) + sqr(v.y)));
}

double WarpingMath::euclideanNorm(const Point2d &p)
{
	return sqrt(sqr(p.x) + sqr(p.y));
}

double WarpingMath::vTv(Vertex v1, Vertex v2)
{
	return (double) (v1.x * v2.x + v1.y * v2.y);
}

float WarpingMath::round (float f)
{
	return floor(f + 0.5);
}

double WarpingMath::round (double d)
{
	return floor(d + 0.5);
}

double WarpingMath::area(Quad &q)
{
	double area1, area2;
	double a1, b1, c1, a2, b2, c2;
	double s1, s2;

	a1 = euclideanNorm(q.v1 - q.v3);
	b1 = euclideanNorm(q.v3 - q.v4);
	c1 = euclideanNorm(q.v1 - q.v4);
	
	a2 = euclideanNorm(q.v2 - q.v4);
	b2 = euclideanNorm(q.v2 - q.v1);
	c2 = c1;

	s1 = (a1 + b1 + c1) / 2.0;
	s2 = (a2 + b2 + c2) / 2.0;

	area1 = sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
	area2 = sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));

	return area1 + area2;
}

double WarpingMath::area(Quad &q, Point2f &p)
{
	double area1, area2, area3, area4;
	double a1, b1, c1, a2, b2, c2, a3, b3, c3, a4, b4, c4;
	double s1, s2, s3, s4;

	// triangle v1, v3, p
	a1 = euclideanNorm(Point2d(q.v1.x - p.x, q.v1.y - p.y));
	b1 = euclideanNorm(Point2d(q.v3.x - p.x, q.v3.y - p.y));
	c1 = euclideanNorm(q.v1 - q.v3);

	// triangle v1, v2, p
	a2 = a1;
	b2 = euclideanNorm(Point2d(q.v2.x - p.x, q.v2.y - p.y));
	c2 = euclideanNorm(q.v1 - q.v2);

	// triangle v2, v4, p
	a3 = b2;
	b3 = euclideanNorm(Point2d(q.v4.x - p.x, q.v4.y - p.y));
	c3 = euclideanNorm(q.v2 - q.v4);

	// tirangle v4, v3, p
	a4 = b1;
	b4 = b3;
	c4 = euclideanNorm(q.v4 - q.v3);

	s1 = (a1 + b1 + c1) / 2.0;
	s2 = (a2 + b2 + c2) / 2.0;
	s3 = (a3 + b3 + c3) / 2.0;
	s4 = (a4 + b4 + c4) / 2.0;

	area1 = sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
	area2 = sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));
	area3 = sqrt(s3 * (s3 - a3) * (s3 - b3) * (s3 - c3));
	area4 = sqrt(s4 * (s4 - a4) * (s4 - b4) * (s4 - c4));

	return area1 + area2 + area3 + area4;
}