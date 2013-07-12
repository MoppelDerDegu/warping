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