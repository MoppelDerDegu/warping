#pragma once

#include "stdafx.h"

struct WarpingMath
{
	static float getDistance(Vertex v1, Vertex v2);
	static float getAverageSaliency(int sumOfSaliencyValues, int numOfPixel);
	static float normalize(float value, float max); // normalizes values to [0...1]
	static double euclideanNorm(const Vertex &v);
	static float round (float f);
	static double round (double d);
	static double vTv(Vertex v1, Vertex v2); // v^tr * v
};

