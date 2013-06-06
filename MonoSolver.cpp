#include "MonoSolver.h"
#include "Helper.h"
#include "QuadSaliencyManager.h"
#include "WarpingMath.h"
#include "FileManager.h"
#include "lib/nlopt-2.3-dll/nlopt.hpp"
#include "MeshManager.h"

MonoSolver::MonoSolver(Size &originalSize)
{
	this->iterationCount = 0;
	this->oldSize = originalSize;
}

MonoSolver::~MonoSolver(void)
{
}

Mesh MonoSolver::getDeformedMesh()
{
	return deformedMesh;
}

Mesh MonoSolver::getInitialGuess()
{
	return tmp;
}

Mesh MonoSolver::solveImageProblem(Mesh &contentAwareMesh, Mesh &originalMesh, Size &newSize, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving image optimization problem..." << endl;

	this->originalMesh = originalMesh;
	this->contentAwareMesh = contentAwareMesh;
	this->saliencyWeightMapping = wfMap;
	this->newSize = newSize;

	MeshManager* mm = MeshManager::getInstance();

	// initial guess is stored in tmp
	initialGuess(contentAwareMesh, tmp, newSize, oldSize);
	
	// copy initial guess to resultmesh
	deformedMesh = mm->deepCopyMesh(tmp);
	vector<double> x = mm->meshToDoubleVec(deformedMesh);

	// formulate optimization problem:

	// derivative free optimization algorithm
	nlopt::opt opt(nlopt::LN_NELDERMEAD, x.size());

	// lower and upper bounds of vertex coordinates
	vector<double> lb = computeLowerImageBoundConstraints(x, newSize);
	vector<double> ub = computeUpperImageBoundConstraints(x, newSize);
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	// minimize objective function
	opt.set_min_objective(MonoSolver::wrapperImageObjectiveFunc, this);

	// convergence criteria
	opt.set_xtol_abs(1);

	double minf;

	for (int i = 0; i < 20; i++)
	{
		cout << "\n>>Solving problem for step " << i + 1 << endl;
		
		calculateEdgeLengthRatios(tmp, deformedMesh, edgeLengthRatios);
		calculateOptimalScaleFactors(originalMesh, deformedMesh, scalingFactors);
		
		nlopt::result result = opt.optimize(x, minf);
	
		cout << "\n>> Solution found after " << iterationCount << " iterations" << endl;
		iterationCount = 0;
	}

	return deformedMesh;
}

double MonoSolver::wrapperImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	MonoSolver* solv = reinterpret_cast<MonoSolver*> (my_func_data);
	return solv->imageObjFunc(x, grad);
}

double MonoSolver::imageObjFunc(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	if (!grad.empty())
	{
		// compute gradient here
	}
	MeshManager* mm = MeshManager::getInstance();

	mm->doubleVecToMesh(x, deformedMesh);

	//double edgeEnergy = totalEdgeEnergy(originalMesh, deformedMesh, edgeLengthRatios);
	double quadEnergy = totalQuadEnergy(originalMesh, deformedMesh, scalingFactors, saliencyWeightMapping);

	double res = /*(0.5 * edgeEnergy) +*/ quadEnergy;

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << res << ends;

	return res;
}

Mesh MonoSolver::redistributeQuads(Mesh &originalMesh, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving optimization problem to redistribute quads..." << endl;
	
	this->deformedMesh = originalMesh;
	this->saliencyWeightMapping = wfMap;
	
	QuadSaliencyManager* qsm = QuadSaliencyManager::getInstance();
	this->edgeSaliency = qsm->assignSaliencyValuesToEdges(originalMesh, saliencyWeightMapping, oldSize);

	MeshManager* mm = MeshManager::getInstance();

	vector<double> x = mm->meshToDoubleVec(deformedMesh);

	vector<double> lb = computeLowerImageBoundConstraints(x, oldSize);
	vector<double> ub = computeUpperImageBoundConstraints(x, oldSize);

	nlopt::opt opt(nlopt::LN_PRAXIS, x.size());

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	opt.set_min_objective(MonoSolver::wrapperRedistributeObjectiveFunc, this);

	opt.set_xtol_abs(1);

	double minf;

	nlopt::result res = opt.optimize(x, minf);
	
	cout << "\n>> Solution found after " << iterationCount << " iterations" << endl;

	iterationCount = 0;
	return deformedMesh;
}

double MonoSolver::wrapperRedistributeObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	MonoSolver* solver = reinterpret_cast<MonoSolver*> (my_func_data);
	return solver->redistributeObjFunc(x, grad);
}

double MonoSolver::redistributeObjFunc(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	if (!grad.empty())
	{
		// compute gradient here
	}

	MeshManager* mm = MeshManager::getInstance();

	mm->doubleVecToMesh(x, deformedMesh);
	double res = totalRedistributionEnergy(deformedMesh, edgeSaliency);

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << res << ends;

	return res;
}