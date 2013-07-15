#include "PathlineOptimizer.h"
#include "WarpingMath.h"
#include "PathlineManager.h"

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

double PathlineOptimizer::pathlineScalingEnergy(PathlineMatrixMapping &singleMap, NeighborMatrixMapping &doubleMap, PathlineTransVecMapping &tmap)
{
	double res = 0.0;
	PathlineManager* pm = PathlineManager::getInstance();

	for (unsigned int k = 0; k < adjacencies.neighbors.size(); k++)
	{
		for (unsigned int l = 0; l < originalSets.pathlines.size(); l++)
		{
			vector<Pathline> pathlineSet = originalSets.pathlines.at(l);
			int firstFrame = pathlineSet.at(0).path.at(0).first;
			int lastFrame = pathlineSet.at(0).path.at(pathlineSet.at(0).path.size() - 1).first;

			pair<Pathline, Pathline> neighbors = pm->getNeighbors(adjacencies.neighbors.at(k), pathlineSet);

			for (unsigned int m = firstFrame - 1; m < lastFrame; m++)
			{
				ScalingMatrix2x2 si, sj, sij;
				TranslationVector2 ti, tj;

				si = getMatrix(singleMap.mapping.at(l), neighbors.first);
				sj = getMatrix(singleMap.mapping.at(l), neighbors.second);

				sij = getMatrix(doubleMap.mapping.at(l), adjacencies.neighbors.at(k));

				ti = getTranslationVector(tmap.mapping.at(l), neighbors.first);
				tj = getTranslationVector(tmap.mapping.at(l), neighbors.second);

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
				ScalingMatrix2x2 si = getMatrix(mmap.mapping.at(k), p);
				TranslationVector2 ti = getTranslationVector(tmap.mapping.at(k), p);

				// si * pi_m + ti
				tmpi.x = si.vx * p.path.at(m).second.x + ti.x;
				tmpi.y = si.vy * p.path.at(m).second.y + ti.y;

				Point2d tmp;

				// (si * pi_m + ti) - qi_m
				tmp.x = tmpi.x - q.path.at(m).second.x;
				tmp.y = tmpi.y - q.path.at(m).second.y;

				res += sqr(WarpingMath::euclideanNorm(tmp));
			}
		}
	}

	return res;
}

PathlineSets PathlineOptimizer::optimizePathlines()
{
	cout << "Optimizing Pathlines..." << endl;

	PathlineSets result;
	PathlineManager* pm = PathlineManager::getInstance();

	PathlineMatrixMapping matMapping;
	PathlineTransVecMapping vecMapping;

	// initialize mappings
	pm->createPathlineMatrixMapping(originalSets, matMapping);
	pm->createPathlineTransVecMapping(originalSets, vecMapping);

	return result;
}

double PathlineOptimizer::wrapperPathlineObjFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	PathlineOptimizer* opt = reinterpret_cast<PathlineOptimizer*> (my_func_data);
	return opt->totalPathlineEnergy(x, grad);
}

double PathlineOptimizer::totalPathlineEnergy(const vector<double> &x, vector<double> &grad)
{
	double energy = 0.0;

	// compute total Energy

	return energy;
}

ScalingMatrix2x2 PathlineOptimizer::getMatrix(vector<pair<Pathline, ScalingMatrix2x2>> &mapping, Pathline &pl)
{
	ScalingMatrix2x2 result;

	for (unsigned int i = 0; i < mapping.size(); i++)
	{
		pair<Pathline, ScalingMatrix2x2> &pair = mapping.at(i);
		
		if (pair.first == pl)
		{
			result = pair.second;
			break;
		}
	}

	return result;
}

ScalingMatrix2x2 PathlineOptimizer::getMatrix(vector<pair<pair<unsigned int, unsigned int>, ScalingMatrix2x2>> &mapping, pair<unsigned int, unsigned int> &neighbors)
{
	ScalingMatrix2x2 result;

	for (unsigned int i = 0; i < mapping.size(); i++)
	{
		pair<pair<unsigned int, unsigned int>, ScalingMatrix2x2> &pair = mapping.at(i);
		
		if (pair.first == neighbors)
		{
			result = pair.second;
			break;
		}
	}

	return result;
}

TranslationVector2 PathlineOptimizer::getTranslationVector(vector<pair<Pathline, TranslationVector2>> &mapping, Pathline &pl)
{
	TranslationVector2 result;

	for (unsigned int i = 0; i < mapping.size(); i++)
	{
		pair<Pathline, TranslationVector2> &pair = mapping.at(i);
		
		if (pair.first == pl)
		{
			result = pair.second;
			break;
		}
	}

	return result;
}

int PathlineOptimizer::getNumberOfDummyVariables(PathlineAdjacencies &adjacencies)
{
	// #neighbors * 2 because we have two variables for each pathline, i.e. the entries in the diagonal 2x2 scaling matrix
	return adjacencies.neighbors.size() * 2;
}