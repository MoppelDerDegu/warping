#pragma 

#include "solver.h"

class PathlineOptimizer : public Solver
{
public:
	PathlineOptimizer(PathlineSets &originalSets, PathlineSets &deformedSets, PathlineAdjacencies &adjacencies);
	~PathlineOptimizer(void);

	void optimizePathlines(PathlineSets &result);

private:

	//pathlines sets for original and deformed videos respectively containing the pathlines for either the left or right view
	PathlineSets originalSets, deformedSets;
	PathlineAdjacencies adjacencies;

	PathlineMatrixMapping optimizedMatMapping;
	PathlineTransVecMapping optimizedVecMapping;
	NeighborMatrixMapping optimizedNeighborMapping;

	// maps the index of the pathline set from all pathline sets to the number of variables used for this pathline set during the optimization
	map<int, int> pathlineSetVariableMapping;
	
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