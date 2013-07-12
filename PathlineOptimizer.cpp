#include "PathlineOptimizer.h"
#include "WarpingMath.h"

PathlineOptimizer::PathlineOptimizer(PathlineSets &originalSets, PathlineSets &deformedSets, PathlineAdjacencies &adjacencies)
{
	this->iterationCount = 0;
	this->originalSets = originalSets;
	this->deformedSets = deformedSets;
	this->adjacencies = adjacencies;
}

PathlineOptimizer::~PathlineOptimizer(void)
{
}

bool PathlineOptimizer::comparePathlines(Pathline &p1, Pathline &p2)
{
	return p1.seedIndex < p2.seedIndex;
}

bool PathlineOptimizer::compareNeighbors(pair<unsigned int, unsigned int> &n1, pair<unsigned int, unsigned int> &n2)
{
	if (n1.first < n2.first)
		return true;
	else if (n1.first > n2.first)
		return false;
	else
		return n1.second < n2.second;
}

double PathlineOptimizer::pathlineScalingEnergy(PathlineMatrixMapping &singleMap, NeighborMatrixMapping &doubleMap, PathlineTransVecMapping &tmap)
{
	double res = 0.0;

	for (unsigned int k = 0; k < adjacencies.neighbors.size(); k++)
	{
		for (unsigned int l = 0; l < originalSets.pathlines.size(); l++)
		{
			vector<Pathline> pathlineSet = originalSets.pathlines.at(l);
			int firstFrame = pathlineSet.at(0).path.at(0).first;
			int lastFrame = pathlineSet.at(0).path.at(pathlineSet.at(0).path.size() - 1).first;

			pair<Pathline, Pathline> neighbors = getNeighbors(adjacencies.neighbors.at(k), pathlineSet);

			for (unsigned int m = firstFrame - 1; m < lastFrame; m++)
			{
				ScalingMatrix2x2 si, sj, sij;
				TranslationVector2 ti, tj;

				si = singleMap.mapping.at(l).at(neighbors.first);
				sj = singleMap.mapping.at(l).at(neighbors.second);

				sij = doubleMap.mapping.at(l).at(adjacencies.neighbors.at(k));

				ti = tmap.mapping.at(l).at(neighbors.first);
				tj = tmap.mapping.at(l).at(neighbors.second);

				Point2d tmpi, tmpj, tmpij, pij;

				// si * pi_t + ti
				tmpi.x = si.vx * neighbors.first.path.at(m).second.x + ti.x;
				tmpi.y = si.vy * neighbors.first.path.at(m).second.y + ti.y;

				// sj * pj_t + tj
				tmpj.x = sj.vx * neighbors.second.path.at(m).second.x + tj.x;
				tmpj.y = sj.vy * neighbors.second.path.at(m).second.y + tj.y;

				// pij = pi - pj
				pij.x = neighbors.first.path.at(m).second.x - neighbors.second.path.at(m).second.x;
				pij.y = neighbors.first.path.at(m).second.y - neighbors.second.path.at(m).second.y;

				// sij * pij
				tmpij.x = sij.vx * pij.x;
				tmpij.x = sij.vy * pij.y;

				res += sqr(WarpingMath::euclideanNorm(tmpi - tmpj - tmpij));
			}
		}
	}

	return res;
}

double PathlineOptimizer::pathlineDeformationEnergy(PathlineMatrixMapping &mmap, PathlineTransVecMapping &tmap)
{
	double res = 0.0;

	for (unsigned int k = 0; k < originalSets.pathlines.size(); k++)
	{
		vector<Pathline> pathlines = originalSets.pathlines.at(k);
		vector<Pathline> deformedPathlines = deformedSets.pathlines.at(k);

		int firstFrame = pathlines.at(0).path.at(0).first;
		int lastFrame = pathlines.at(0).path.at(pathlines.at(0).path.size() - 1).first;

		for (unsigned int l = 0; l < pathlines.size(); l++)
		{
			Pathline p = pathlines.at(l);
			Pathline q = deformedPathlines.at(l);

			for (unsigned int m = firstFrame - 1; m < lastFrame; m++)
			{
				Point2d tmpi;
				ScalingMatrix2x2 si = mmap.mapping.at(k).at(p);
				TranslationVector2 ti = tmap.mapping.at(k).at(p);

				// sj * pi_m + ti
				tmpi.x = si.vx * p.path.at(m).second.x + ti.x;
				tmpi.y = si.vy * p.path.at(m).second.y + ti.y;

				Point2d tmp;

				// (sj * pi_m + ti) - qi_m
				tmp.x = tmpi.x - q.path.at(m).second.x;
				tmp.y = tmpi.y - q.path.at(m).second.y;

				res += sqr(WarpingMath::euclideanNorm(tmp));
			}
		}
	}

	return res;
}

pair<Pathline, Pathline> PathlineOptimizer::getNeighbors(pair<unsigned int, unsigned int> &neighbors, vector<Pathline> &pathlines)
{
	pair<Pathline, Pathline> pair;
	
	for (unsigned int k = 0; k < pathlines.size(); k++)
	{
		if (pathlines.at(k).seedIndex == neighbors.first)
		{
			pair.first = pathlines.at(k);
			break;
		}
	}

	for (unsigned int k = 0; k < pathlines.size(); k++)
	{
		if (pathlines.at(k).seedIndex == neighbors.second)
		{
			pair.second = pathlines.at(k);
			break;
		}
	}

	return pair;
}