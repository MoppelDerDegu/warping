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

vector<pair<Edge, float>> QuadSaliencyManager::assignSaliencyValuesToEdges(Mesh &m, vector<pair<float, Quad>> saliencyValues, Size &size)
{
	vector<pair<Edge, float>> map;
	
	for (unsigned int i = 0; i < m.edges.size(); i++)
	{
		Edge e = m.edges.at(i);
		bool isBorder = isEdgeOnBorder(e, size);
		vector<Quad> tmpQuads;
		
		// check which quads belong to an edge
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

			// edges in opposite direction since we are not interested in the edge direction
			Edge _a, _b, _c, _d;
			_a.src = q.v2;
			_a.dest = q.v1;
			_b.src = q.v4;
			_b.dest = q.v2;
			_c.src = q.v3;
			_c.dest = q.v4;
			_d.src = q.v1;
			_d.dest = q.v3;


			// quad belongs to edge
			if (e == a || e == b || e == c || e == d || e == _a || e == _b || e == _c || e == _d)
			{
				if(isBorder)
				{	
					// only one quad belongs to this edge
					for (unsigned int k = 0; k < saliencyValues.size(); k++)
					{
						if (saliencyValues.at(k).second == q)
						{
							//get saliency of this quad
							pair<Edge, float> p;
							p.first = e;
							p.second = saliencyValues.at(k).first;
							map.push_back(p);
							break;
						}
					}
					break;
				}
				else
				{
					// two edges belong to this quad
					if (tmpQuads.empty())
						tmpQuads.push_back(q);
					else 
					{
						tmpQuads.push_back(q);
						vector<float> tmpSaliency;
						for (unsigned int k = 0; k < saliencyValues.size(); k++)
						{
							// get the saliency values of these two quads
							if (saliencyValues.at(k).second == tmpQuads.at(0))
								tmpSaliency.push_back(saliencyValues.at(k).first);
							else if ((saliencyValues.at(k).second == tmpQuads.at(1)))
								tmpSaliency.push_back(saliencyValues.at(k).first);

							if (tmpSaliency.size() == 2)
							{
								// assign the average saliency of these two quads to the edge
								float avg = (tmpSaliency.at(0) + tmpSaliency.at(1)) / 2.0;
								pair<Edge, float> p;
								p.first = e;
								p.second = avg;
								map.push_back(p);
								break;
							}
						}
						break;
					}
				}
			}
		}
	}

	return map;
}

bool QuadSaliencyManager::isEdgeOnBorder(Edge &e, Size &size)
{
	return (e.src.x == 0 && e.dest.x == 0) || (e.src.y == 0 && e.dest.y == 0) || (e.src.x == size.width && e.dest.x == size.width) || (e.src.y == size.height && e.dest.y == size.height);
}