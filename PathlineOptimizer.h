#pragma 

#include "solver.h"

typedef struct PathlineMatrixMapping
{
	// vector specifies the different sets of pathlines
	vector<map<Pathline, ScalingMatrix2x2>> mapping;
};

typedef struct PathlineTransVecMapping
{
	// vector specifies the different sets of pathlines
	vector<map<Pathline, TranslationVector2>> mapping;
};

typedef struct NeighborMatrixMapping
{
	vector<map<pair<unsigned int, unsigned int>, ScalingMatrix2x2>> mapping;
};

class PathlineOptimizer : public Solver
{
public:
	PathlineOptimizer(PathlineSets &originalSets, PathlineSets &deformedSets, PathlineAdjacencies &adjacencies);
	~PathlineOptimizer(void);

private:
	PathlineSets originalSets, deformedSets;
	PathlineAdjacencies adjacencies;

	bool comparePathlines(Pathline &p1, Pathline &p2);
	bool compareNeighbors(pair<unsigned int, unsigned int> &n1, pair<unsigned int, unsigned int> &n2);

	pair<Pathline, Pathline> getNeighbors(pair<unsigned int, unsigned int> &neighbors, vector<Pathline> &pathlines);

	double pathlineScalingEnergy(PathlineMatrixMapping &singleMap, NeighborMatrixMapping &doubleMap, PathlineTransVecMapping &tmap);
	double pathlineDeformationEnergy(PathlineMatrixMapping &mmap, PathlineTransVecMapping &tmap);
};