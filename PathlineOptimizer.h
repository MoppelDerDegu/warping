#pragma 

#include "solver.h"

typedef struct PathlineMatrixMapping
{
	// outer vector specifies the different sets of pathlines
	// inner vector specifies the pathline <-> scaling matrix mapping
	vector<vector<pair<Pathline, ScalingMatrix2x2>>> mapping;
};

typedef struct PathlineTransVecMapping
{
	// vector specifies the different sets of pathlines
	// inner vector specifies the pathline <-> translation vector mapping
	vector<vector<pair<Pathline, TranslationVector2>>> mapping;
};

typedef struct NeighborMatrixMapping
{
	// vector specifies the different sets of pathlines
	// inner vector specifies the neighbor indices <-> scaling matrix mapping
	vector<vector<pair<pair<unsigned int, unsigned int>, ScalingMatrix2x2>>> mapping;
};

class PathlineOptimizer : public Solver
{
public:
	PathlineOptimizer(PathlineSets &originalSets, PathlineSets &deformedSets, PathlineAdjacencies &adjacencies);
	~PathlineOptimizer(void);

	PathlineSets optimizePathlines();

private:

	//pathlines sets for original and deformed videos respectively containing the pathlines for either the left or right view
	PathlineSets originalSets, deformedSets;
	PathlineAdjacencies adjacencies;

	bool comparePathlines(Pathline &p1, Pathline &p2);
	bool compareNeighbors(pair<unsigned int, unsigned int> &n1, pair<unsigned int, unsigned int> &n2);
	ScalingMatrix2x2 getMatrix(vector<pair<Pathline, ScalingMatrix2x2>> &mapping, Pathline &pl);
	ScalingMatrix2x2 getMatrix(vector<pair<pair<unsigned int, unsigned int>, ScalingMatrix2x2>> &mapping, pair<unsigned int, unsigned int> &neighbors);
	TranslationVector2 getTranslationVector(vector<pair<Pathline, TranslationVector2>> &mapping, Pathline &pl);

	pair<Pathline, Pathline> getNeighbors(pair<unsigned int, unsigned int> &neighbors, vector<Pathline> &pathlines);

	static double wrapperPathlineObjFunc(const vector<double> &x, vector<double> &grad, void *my_func_data);

	double pathlineScalingEnergy(PathlineMatrixMapping &singleMap, NeighborMatrixMapping &doubleMap, PathlineTransVecMapping &tmap);
	double pathlineDeformationEnergy(PathlineMatrixMapping &mmap, PathlineTransVecMapping &tmap);

	double totalPathlineEnergy(const vector<double> &x, vector<double> &grad);
};