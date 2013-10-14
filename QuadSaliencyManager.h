#pragma once

#include "stdafx.h"

// manager class that handles the saliency of quads
class QuadSaliencyManager
{
public:
	~QuadSaliencyManager()
	{
		instanceFlag = false;
	}

	static QuadSaliencyManager* getInstance();

	vector<pair<float, Quad>> assignSaliencyValuesToQuads(Mesh &m, Mat &saliencyMap);

private:
	static bool instanceFlag;
	static QuadSaliencyManager* single;
	QuadSaliencyManager()
	{
	}

	bool isEdgeOnBorder(Edge &e, Size &size);
	bool isPixelInQuad(Quad &quad, Point2i &point);
};

