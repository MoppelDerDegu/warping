#pragma once

#include "stdafx.h"

class QuadSaliencyManager
{
public:
	QuadSaliencyManager(void);
	~QuadSaliencyManager(void);
	vector<pair<float, Quad>> assignSaliencyValuesToQuads(Mesh &m, Mat &saliencyMap);
	vector<pair<Edge, float>> assignSaliencyValuesToEdges(Mesh &m, vector<pair<float, Quad>> saliencyValues, Size &size);
private:
	bool isEdgeOnBorder(Edge &e, Size &size);
	bool isPixelInQuad(Quad &quad, Point2i &point);
};

