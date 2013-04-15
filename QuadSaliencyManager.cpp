#include "QuadSaliencyManager.h"
#include "Helper.h"

QuadSaliencyManager::QuadSaliencyManager(void)
{
}


QuadSaliencyManager::~QuadSaliencyManager(void)
{
}

vector<pair<float, Quad>> QuadSaliencyManager::assignSaliencyValuesToQuads(Mesh &m, Mat &saliencyMap)
{
	vector<pair<float, Quad>> result;
	int x, y;
	float quadmax = 0;
	int roiWidth, roiHeight;
	Mat imageROI;

	for (unsigned int i = 0; i < m.quads.size(); i++)
	{
		int sum = 0;
		float max = 0;
		pair<float, Quad> pair;

		x = m.quads.at(i).v1.x;
		y = m.quads.at(i).v1.y;

		roiWidth = (int) Helper::getDistance(m.quads.at(i).v1, m.quads.at(i).v2);
		roiHeight = (int) Helper::getDistance(m.quads.at(i).v1, m.quads.at(i).v3);

		imageROI = saliencyMap(Rect(x, y, roiWidth, roiHeight));
		
		for (int j = 0; j < imageROI.rows; j++)
		{
			for (int k = 0; k < imageROI.cols; k++)
			{
				float value = imageROI.at<float> (j, k);

				if (value > max)
					max = value;

				sum += value;
			}
		}

		float wf = Helper::getAverageSaliency(sum, roiWidth * roiHeight);
		pair.first = wf;
		pair.second = m.quads.at(i);
		result.push_back(pair);
	}

	// determine max saliency values of all quads
	for (unsigned int i = 0; i < result.size(); i++)
	{
		float wf = result.at(i).first;

		if (wf > quadmax)
			quadmax = wf;
	}

	// normalize saliency weight factors
	for (unsigned int i = 0; i < result.size(); i++)
	{
		float value = result.at(i).first;
		float wf = Helper::normalize(value, quadmax);
		result.at(i).first = wf;
	}

	return result;
}
