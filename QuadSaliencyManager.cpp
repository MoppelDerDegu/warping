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
	int roiWidth, roiHeight;
	Mat imageROI;
	MatIterator_<unsigned char> it, it_end;

	for (int i = 0; i < m.quads.size(); i++)
	{
		int sum = 0;
		pair<float, Quad> pair;

		x = m.quads.at(i).v1.first;
		y = m.quads.at(i).v1.second;

		roiWidth = (int) Helper.getDistance(m.quads.at(i).v1, m.quads.at(i).v2);
		roiHeight = (int) Helper.getDistance(m.quads.at(i).v1, m.quads.at(i).v3);

		imageROI = saliencyMap(Rect(x, y, roiWidth, roiHeight));

		it = imageROI.begin<unsigned char>();
		it_end = imageROI.end<unsigned char>();

		while(it != it_end)
		{
			sum = sum + (*it);
			it++;
		}

		float wf = Helper.normalize(Helper.getAverageSaliency(sum, roiWidth * roiHeight));
		pair.first = wf;
		pair.second = m.quads.at(i);
		result.push_back(pair);
	}

	return result;
}
