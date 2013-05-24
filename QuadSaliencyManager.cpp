#include "QuadSaliencyManager.h"
#include "Helper.h"
#include "WarpingMath.h"

QuadSaliencyManager::QuadSaliencyManager(void)
{
}


QuadSaliencyManager::~QuadSaliencyManager(void)
{
}

vector<pair<float, Quad>> QuadSaliencyManager::assignSaliencyValuesToQuads(Mesh &m, Mat &saliencyMap)
{
	cout << ">> Assigning saliency weights to mesh" << endl;

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

		roiWidth = (int) WarpingMath::getDistance(m.quads.at(i).v1, m.quads.at(i).v2);
		roiHeight = (int) WarpingMath::getDistance(m.quads.at(i).v1, m.quads.at(i).v3);

		imageROI = saliencyMap(Rect(x, y, roiWidth, roiHeight));
		
		for (int j = 0; j < imageROI.rows; j++)
		{
			for (int k = 0; k < imageROI.cols; k++)
			{
				uchar value = imageROI.at<uchar> (j, k);

				if (value > max)
					max = value;

				sum += value;
			}
		}

		float wf = WarpingMath::getAverageSaliency(sum, roiWidth * roiHeight);
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
		float wf = WarpingMath::normalize(value, quadmax);
		result.at(i).first = wf;
	}
	
	return result;
}

map<Edge, float> QuadSaliencyManager::assignSaliencyValuesToEdges(Mesh &m, vector<pair<float, Quad>> saliencyValues, Size &size)
{
	map<Edge, float> map;
	
	for (unsigned int i = 0; i < m.edges.size(); i++)
	{
		Edge e = m.edges.at(i);
		bool isBorder = isEdgeOnBorder(e, size);

		for (unsigned int j = 0; j < m.quads.size(); j++)
		{
			Quad q = m.quads.at(j);
			Edge a, b, c, d;
			a.src = q.v1;
			a.dest = q.v2;
			b.src = q.v2;
			b.dest = q.v4;
			c.src = q.v4;
			c.dest = q.v3;
			d.src = q.v3;
			d.dest = q.v1;

			if (e == a)
			{
			}
			else if (e == b)
			{
			}
			else if (e == c)
			{
			}
			else if (e == d)
			{
			}

		}
	}

	return map;
}

bool QuadSaliencyManager::isEdgeOnBorder(Edge &e, Size &size)
{
	return (e.src.x == 0 && e.dest.x == 0) || (e.src.y == 0 && e.dest.y == 0) || (e.src.x == size.width && e.dest.x == size.width) || (e.src.y == size.height && e.dest.y == size.height);
}