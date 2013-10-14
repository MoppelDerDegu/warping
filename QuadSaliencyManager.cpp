#include "QuadSaliencyManager.h"
#include "Helper.h"
#include "WarpingMath.h"

bool QuadSaliencyManager::instanceFlag = false;
QuadSaliencyManager* QuadSaliencyManager::single = NULL;

QuadSaliencyManager* QuadSaliencyManager::getInstance()
{
	if (!instanceFlag)
	{
		single = new QuadSaliencyManager();
		instanceFlag = true;
	}
	
	return single;
}

vector<pair<float, Quad>> QuadSaliencyManager::assignSaliencyValuesToQuads(Mesh &m, Mat &saliencyMap)
{
	cout << ">> Assigning saliency weights to mesh" << endl;

	vector<pair<float, Quad>> result;
	float quadmax = 0;
	Mat imageROI;

	// assign saliency values to quads
	for (unsigned int i = 0; i < m.quads.size(); i++)
	{
		int sum = 0;
		int pixelCounter = 0;
		float max = 0;
		pair<float, Quad> pair;
		Quad relative = Helper::getRelativeCoordinates(m.quads.at(i));

		// place ROI over quad
		Helper::getImageROI(m.quads.at(i), imageROI, saliencyMap);
		
		for (int j = 0; j < imageROI.rows; j++)
		{
			for (int k = 0; k < imageROI.cols; k++)
			{
				Point2i p(j, k);

				if (isPixelInQuad(relative, p))
				{
					// read saliency value
					uchar value = imageROI.at<uchar> (j, k);

					if (value > max)
						max = value;

					sum += value;
					pixelCounter++;
				}
				else
					continue;
			}
		}

		float wf = WarpingMath::getAverageSaliency(sum, pixelCounter);
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
	
	// set minimum saliency weight to 0.1
	for (unsigned int i = 0; i < result.size(); i++)
	{
		float value = result.at(i).first;
		
		if (value < 0.1)
			value = 0.1;
	}

	return result;
}

bool QuadSaliencyManager::isEdgeOnBorder(Edge &e, Size &size)
{
	return (e.src.x == 0 && e.dest.x == 0) || (e.src.y == 0 && e.dest.y == 0) || (e.src.x == size.width && e.dest.x == size.width) || (e.src.y == size.height && e.dest.y == size.height);
}

bool QuadSaliencyManager::isPixelInQuad(Quad &quad, Point2i &point)
{
	Vertex u, v, a, b, p, q;
	u = quad.v3;
	v = quad.v2;
	a = quad.v4 - quad.v3;
	b = quad.v1 - quad.v3;
	p = quad.v1 - quad.v2;
	q = quad.v4 - quad.v2;

	double r = (double) (b.y * (point.x - u.x) + b.x * (u.y - point.y)) / (double) (b.y * a.x - b.x * a.y);
	double s = (double) (point.y - r * a.y - u.y) / (double) b.y;

	if (!(r < 0.0 || r > 1.0 || s < 0.0 || s > 1.0))
	{
		if (r + s < 1.0)
			return true;
		else
		{
			double _r = (double) (q.y * (point.x - v.x) + q.x * (v.y - point.y)) / (double) (q.y * p.x - q.x * p.y);
			double _s = (double) (point.y - _r * p.y - v.y) / (double) q.y;

			if (_r + _s <= 1 && !(_r < 0.0 || _r > 1.0 || _s < 0.0 || _s > 1.0))
				return true;
			else
				return false;
		}
	}
	else
	{
		double _r = (double) (q.y * (point.x - v.x) + q.x * (v.y - point.y)) / (double) (q.y * p.x - q.x * p.y);
		double _s = (double) (point.y - _r * p.y - v.y) / (double) q.y;

		if (_r + _s <= 1 && !(_r < 0.0 || _r > 1.0 || _s < 0.0 || _s > 1.0))
			return true;
		else
			return false;
	}
}