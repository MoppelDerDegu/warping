#pragma 

#include "solver.h"
#include <boost\thread.hpp>

class PathlineOptimizer : public Solver
{
public:
	PathlineOptimizer(PathlineSets &originalSets, PathlineSets &deformedSets, PathlineAdjacencies &adjacencies, Size &oldSize, Size &newSize);
	~PathlineOptimizer(void);

	// wrapper method to start the thread
	void optimizePathlines();
	// join the thread
	void join();

	PathlineSets getResult();

private:
	boost::thread myThread; // thread objective that contains the execution of the optimization

	Size newSize, oldSize;

	//pathlines sets for original and deformed videos respectively containing the pathlines for either the left or right view
	PathlineSets originalSets, deformedSets;
	PathlineAdjacencies adjacencies;
	
	// the final optimized pathlines
	PathlineSets result;

	// mappings for data structures
	PathlineMatrixMapping optimizedMatMapping;
	PathlineTransVecMapping optimizedVecMapping;
	NeighborMatrixMapping optimizedNeighborMapping;

	// actual optimization
	void _optimizePathlines();

	// maps the index of the pathline set from all pathline sets to the number of variables used for this pathline set during the optimization
	map<int, int> pathlineSetVariableMapping;
	
	void constructOptimizedPathlineSets(PathlineSets &result);

	// returns number of dummy variables for the scaling matrix s_ij between two neighboring pathlines p_i and p_j used during optimization
	int getNumberOfDummyVariables(PathlineAdjacencies &adjacencies);

	void createAllVariables(PathlineMatrixMapping &matMapping, PathlineTransVecMapping &vecMapping, vector<double> &allVariables); // creates double variables from the different data structures

	static double wrapperPathlineObjFunc(const vector<double> &x, vector<double> &grad, void *my_func_data); // wrapper for the objective function that is passed to the NLopt library

	double pathlineScalingEnergy(PathlineMatrixMapping &singleMap, NeighborMatrixMapping &doubleMap, PathlineTransVecMapping &tmap); // equation 3.13
	double pathlineDeformationEnergy(PathlineMatrixMapping &mmap, PathlineTransVecMapping &tmap); // equation 3.14

	double totalPathlineEnergy(const vector<double> &x, vector<double> &grad); // equation 3.15

	vector<double> computeLowerBounds(vector<double> &variables);

	void doubleVecToMapping(const vector<double> &vars, PathlineMatrixMapping &outMatMapping, PathlineTransVecMapping &outVecMapping, NeighborMatrixMapping &outNeighborMapping);
};