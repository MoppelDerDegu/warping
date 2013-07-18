#include "PathlineOptimizer.h"
#include "WarpingMath.h"
#include "PathlineManager.h"
#include "lib/nlopt-2.3-dll/nlopt.hpp"

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

	for (unsigned int l = 0; l < originalSets.pathlines.size(); l++)
	{
		for (unsigned int k = 0; k < adjacencies.neighbors.size(); k++)
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

				// si * pi_m + ti
				tmpi.x = si.vx * neighbors.first.path.at(m).second.x + ti.x;
				tmpi.y = si.vy * neighbors.first.path.at(m).second.y + ti.y;

				// sj * pj_m + tj
				tmpj.x = sj.vx * neighbors.second.path.at(m).second.x + tj.x;
				tmpj.y = sj.vy * neighbors.second.path.at(m).second.y + tj.y;

				// pij = pi - pj
				pij.x = neighbors.first.path.at(m).second.x - neighbors.second.path.at(m).second.x;
				pij.y = neighbors.first.path.at(m).second.y - neighbors.second.path.at(m).second.y;

				// sij * pij
				tmpij.x = sij.vx * pij.x;
				tmpij.y = sij.vy * pij.y;

				double norm = WarpingMath::euclideanNorm(tmpi - tmpj - tmpij);

				res += sqr(norm);
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

void PathlineOptimizer::optimizePathlines(PathlineSets &result)
{
	std::cout << "\nOptimizing Pathlines..." << endl;

	PathlineManager* pm = PathlineManager::getInstance();

	// initialize mappings
	pm->createPathlineMatrixMapping(originalSets, optimizedMatMapping);
	pm->createPathlineTransVecMapping(originalSets, optimizedVecMapping);
	pm->createNeighborMatrixMapping(originalSets, adjacencies, optimizedNeighborMapping);

	// create variables
	vector<double> x;
	createAllVariables(optimizedMatMapping, optimizedVecMapping, x);

	// lower bounds
	vector<double> lb = computeLowerBounds(x);

	// create optimizer object
	nlopt::opt opt(nlopt::LN_NELDERMEAD, x.size());

	opt.set_lower_bounds(lb);
	opt.set_min_objective(PathlineOptimizer::wrapperPathlineObjFunc, this);

	// convergence criteria
	opt.set_xtol_abs(10.0);

	double minf;

	nlopt::result res = opt.optimize(x, minf);

	std::cout << "\n>> Solution found after " << iterationCount << " iterations" << endl;

	constructOptimizedPathlineSets(result);
}

double PathlineOptimizer::wrapperPathlineObjFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	PathlineOptimizer* opt = reinterpret_cast<PathlineOptimizer*> (my_func_data);
	return opt->totalPathlineEnergy(x, grad);
}

double PathlineOptimizer::totalPathlineEnergy(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	if (!grad.empty())
	{
		// compute gradient here
	}

	double energy = 0.0;
	
	doubleVecToMapping(x, optimizedMatMapping, optimizedVecMapping, optimizedNeighborMapping);

	double first = pathlineScalingEnergy(optimizedMatMapping, optimizedNeighborMapping, optimizedVecMapping);
	double second = pathlineDeformationEnergy(optimizedMatMapping, optimizedVecMapping);

	energy = first + 0.5 * second;

	std::cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << energy << ends;

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

void PathlineOptimizer::createAllVariables(PathlineMatrixMapping &matMapping, PathlineTransVecMapping &vecMapping, vector<double> &allVariables)
{
	allVariables.clear();
	PathlineManager* pm = PathlineManager::getInstance();

	for (unsigned int i = 0; i < originalSets.pathlines.size(); i++)
	{
		vector<pair<Pathline, ScalingMatrix2x2>> &matMap = matMapping.mapping.at(i);
		vector<pair<Pathline, TranslationVector2>> &vecMap = vecMapping.mapping.at(i);
		vector<double> vars;

		int dummies = getNumberOfDummyVariables(adjacencies);

		pm->mappingsToDoubleVec(matMap, vecMap, dummies, vars);

		pathlineSetVariableMapping.insert(pair<int, int>(i, vars.size()));
		allVariables.insert(allVariables.end(), vars.begin(), vars.end());
	}
}

vector<double> PathlineOptimizer::computeLowerBounds(vector<double> &variables)
{
	vector<double> res;

	for (unsigned int i = 0; i < variables.size(); i++)
	{
		res.push_back(0.0);
	}

	return res;
}

void PathlineOptimizer::doubleVecToMapping(const vector<double> &vars, PathlineMatrixMapping &outMatMapping, PathlineTransVecMapping &outVecMapping, NeighborMatrixMapping &outNeighborMapping)
{
	int preOffset = 0;
	int postOffset = 0;

	for (map<int, int>::iterator it = pathlineSetVariableMapping.begin(); it != pathlineSetVariableMapping.end(); ++it)
	{
		postOffset += it->second;

		// extract subset of variables for each pathline set
		vector<double>::const_iterator first = vars.begin() + preOffset;
		vector<double>::const_iterator last = vars.begin() + postOffset;
		vector<double> sub(first, last);

		int matCounter = 0;
		int vecCounter = 0;
		int neighborCounter = 0;
		for (unsigned int i = 0; i < sub.size(); i += 2)
		{
			vector<pair<Pathline, ScalingMatrix2x2>> &matMap = outMatMapping.mapping.at(it->first);
			vector<pair<Pathline, TranslationVector2>> &vecMap = outVecMapping.mapping.at(it->first);
			vector<pair<pair<unsigned int, unsigned int>, ScalingMatrix2x2>> &neighborMap = outNeighborMapping.mapping.at(it->first);

			if (i > (2 * matMap.size()) - 1)
			{
				if (i > ((2 * vecMap.size()) - 1) + 2 * matMap.size())
				{
					// variables are dummy entries
					neighborMap.at(neighborCounter).second.vx = sub.at(i);
					neighborMap.at(neighborCounter).second.vy = sub.at(i + 1);

					neighborMap.at(neighborCounter).first = adjacencies.neighbors.at(neighborCounter);

					neighborCounter++;
				}
				else
				{
					// variables are translation vector entries
					vecMap.at(vecCounter).second.x = sub.at(i);
					vecMap.at(vecCounter).second.y = sub.at(i + 1);

					vecCounter++;
				}
			}
			else
			{
				// variables are scaling matrix entries
				matMap.at(matCounter).second.vx = sub.at(i);
				matMap.at(matCounter).second.vy = sub.at(i + 1);

				matCounter++;
			}
		}

		preOffset = postOffset;
	}
}

void PathlineOptimizer::constructOptimizedPathlineSets(PathlineSets &result)
{
	for (unsigned int i = 0; i < optimizedMatMapping.mapping.size(); i++)
	{
		vector<Pathline> newPathlines;
		
		vector<pair<Pathline, ScalingMatrix2x2>> &matMapping = optimizedMatMapping.mapping.at(i);
		vector<pair<Pathline, TranslationVector2>> &vecMapping = optimizedVecMapping.mapping.at(i);

		for (unsigned int j = 0; j < matMapping.size(); j++)
		{
			ScalingMatrix2x2 &sj = matMapping.at(j).second;
			TranslationVector2 &tj = vecMapping.at(j).second;

			Pathline &oldPathline = matMapping.at(j).first;
			Pathline newPathline;

			newPathline.seedIndex = oldPathline.seedIndex;

			for (unsigned int k = 0; k < oldPathline.path.size(); k++)
			{
				Point2f location;

				// newPathline = sj * oldPathline + tj
				location.x = sj.vx * oldPathline.path.at(k).second.x + tj.x;
				location.y = sj.vy * oldPathline.path.at(k).second.y + tj.y;

				pair<int, Point2f> pair(oldPathline.path.at(k).first, location);

				newPathline.path.push_back(pair);
			}

			newPathlines.push_back(newPathline);
		}

		result.pathlines.push_back(newPathlines);
	}
}