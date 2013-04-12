#pragma once

#include "stdafx.h"

class QuadSaliencyManager
{
public:
	QuadSaliencyManager(void);
	~QuadSaliencyManager(void);
	vector<pair<float, Quad>> assignSaliencyValuesToQuads(Mesh &m, Mat &saliencyMap);
private:
	vector<pair<float, Quad>> wf_values; // contains the average saliency values normalized to [0, 1] for each quad
};

