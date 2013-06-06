#pragma once

#include "stdafx.h"

class QuadSaliencyManager
{
public:
	~QuadSaliencyManager()
	{
		instanceFlag = false;
	}

	static QuadSaliencyManager* getInstance();

	vector<pair<float, Quad>> assignSaliencyValuesToQuads(Mesh &m, Mat &saliencyMap);
	vector<pair<Edge, float>> assignSaliencyValuesToEdges(Mesh &m, vector<pair<float, Quad>> saliencyValues, Size &size);

private:
	static bool instanceFlag;
	static QuadSaliencyManager* single;
	QuadSaliencyManager()
	{
	}

	bool isEdgeOnBorder(Edge &e, Size &size);
	bool isPixelInQuad(Quad &quad, Point2i &point);
};

