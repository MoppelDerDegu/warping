#pragma 

#include "solver.h"

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

	PathlineMatrixMapping optimizedMatMapping;
	PathlineTransVecMapping optimizedVecMapping;
	NeighborMatrixMapping optimizedNeighborMapping;

	// maps the index of the pathline set from all pathline sets to the number of variables used for this pathline set during the optimization
	map<int, int> pathlineSetVariableMapping;

	ScalingMatrix2x2 getMatrix(vector<pair<Pathline, ScalingMatrix2x2>> &mapping, Pathline &pl);
	ScalingMatrix2x2 getMatrix(vector<pair<pair<unsigned int, unsigned int>, ScalingMatrix2x2>> &mapping, pair<unsigned int, unsigned int> &neighbors);
	TranslationVector2 getTranslationVector(vector<pair<Pathline, TranslationVector2>> &mapping, Pathline &pl);

	void constructOptimizedPathlineSets(PathlineSets &result);

	// returns number of dummy variables for the scaling matrix s_ij between two neighboring pathlines p_i and p_j used during optimization
	int getNumberOfDummyVariables(PathlineAdjacencies &adjacencies);

	void createAllVariables(PathlineMatrixMapping &matMapping, PathlineTransVecMapping &vecMapping, vector<double> &allVariables);

	static double wrapperPathlineObjFunc(const vector<double> &x, vector<double> &grad, void *my_func_data);

	double pathlineScalingEnergy(PathlineMatrixMapping &singleMap, NeighborMatrixMapping &doubleMap, PathlineTransVecMapping &tmap);
	double pathlineDeformationEnergy(PathlineMatrixMapping &mmap, PathlineTransVecMapping &tmap);

	double totalPathlineEnergy(const vector<double> &x, vector<double> &grad);

	vector<double> computeLowerBounds(vector<double> &variables);

	void doubleVecToMapping(const vector<double> &vars, PathlineMatrixMapping &outMatMapping, PathlineTransVecMapping &outVecMapping, NeighborMatrixMapping &outNeighborMapping);
};