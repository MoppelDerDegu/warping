#include "StereoSolver.h"
#include "MeshManager.h"
#include "lib/nlopt-2.3-dll/nlopt.hpp"
#include "WarpingMath.h"
#include "FileManager.h"

StereoSolver::StereoSolver(void)
{
	iterationCount = 0;
}

StereoSolver::~StereoSolver(void)
{
}

Mesh StereoSolver::getInitialLeft()
{
	return initialLeft;
}

Mesh StereoSolver::getInitialRight()
{
	return initialRight;
}

pair<Mesh, Mesh> StereoSolver::solveStereoImageProblem(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight)
{
	cout << ">> Solving stereo image optimization problem..." << endl;

	MeshManager* mm = MeshManager::getInstance();

	this->saliencyWeightMappingLeft = wfMapLeft;
	this->saliencyWeightMappingRight = wfMapRight;
	this->originalLeft = originalLeft;
	this->originalRight = originalRight;

	// left size = right size
	Size oldLeftSize = Size(oldSize.width / 2, oldSize.height);
	Size newLeftSize = Size(newSize.width / 2, newSize.height);

	// initial guess for left and right view
	initialGuess(originalLeft, initialLeft, newLeftSize, oldLeftSize);
	initialGuess(originalRight, initialRight, newLeftSize, oldLeftSize);

	// copy initial left and right to result
	deformedLeft = mm->deepCopyMesh(initialLeft);
	deformedRight = mm->deepCopyMesh(initialRight);

	// left mesh
	vector<double> x = mm->meshToDoubleVec(deformedLeft);

	// compute image boundaries
	vector<double> lb = computeLowerImageBoundConstraints(x, newLeftSize);
	lb.insert(lb.end(), lb.begin(), lb.end());
	vector<double> ub = computeUpperImageBoundConstraints(x, newLeftSize);
	ub.insert(ub.end(), ub.begin(), ub.end());

	// right mesh
	vector<double> _x = mm->meshToDoubleVec(deformedRight);
	splitIndex = x.size();
	
	// append the right mesh to the vector containing the left mesh
	x.insert(x.end(), _x.begin(), _x.end());

	// formulate optimization problem:

	// derivative free optimization algorithm
	nlopt::opt opt(nlopt::LN_NELDERMEAD, x.size());

	// minimize objective function
	opt.set_min_objective(StereoSolver::wrapperStereoImageObjectiveFunc, this);

	// convergence criteria
	opt.set_xtol_abs(1);

	// set boundary constraints
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	double minf;

	for (int i = 0; i < 1; i++)
	{
		cout << ">>Solving problem for step " << i + 1 << endl;
		
		calculateEdgeLengthRatios(originalLeft, deformedLeft, edgeLengthRatiosLeft);
		calculateEdgeLengthRatios(originalRight, deformedRight, edgeLengthRatiosRight);

		calculateOptimalScaleFactors(originalLeft, deformedLeft, scalingFactorsLeft);
		calculateOptimalScaleFactors(originalRight, deformedRight, scalingFactorsRight);
		
		nlopt::result result = opt.optimize(x, minf);
	
		cout << "\n>> Solution found after " << iterationCount << " iterations" << endl;
		iterationCount = 0;
	}

	pair<Mesh, Mesh> p(deformedLeft, deformedRight);

	return p;
}

double StereoSolver::wrapperStereoImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	StereoSolver* solver = reinterpret_cast<StereoSolver*> (my_func_data);
	return solver->stereoImageObjFunc(x, grad);
}

double StereoSolver::stereoImageObjFunc(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	if (!grad.empty())
	{
		// compute gradient here
	}

	MeshManager* mm = MeshManager::getInstance();

	// split array into left and right mesh
	vector<double> left(x.begin(), x.begin() + splitIndex);
	vector<double> right(x.begin() + splitIndex, x.end());

	mm->doubleVecToMesh(left, deformedLeft);
	mm->doubleVecToMesh(right, deformedRight);

	double quadEnergyLeft = totalQuadEnergy(originalLeft, deformedLeft, scalingFactorsLeft, saliencyWeightMappingLeft);
	double quadEnergyRight = totalQuadEnergy(originalRight, deformedRight, scalingFactorsRight, saliencyWeightMappingRight);

	double totalQuadEnergy = quadEnergyLeft + quadEnergyRight;

	double edgeEnergyLeft = totalEdgeEnergy(originalLeft, deformedLeft, edgeLengthRatiosLeft);
	double edgeEnergyRight = totalEdgeEnergy(originalRight, deformedRight, edgeLengthRatiosRight);

	double totalEdgeEnergy = edgeEnergyLeft + edgeEnergyRight;

	double disparityEnergy = stereoEnergy(originalLeft, originalRight, deformedLeft, deformedRight);

	double totalEnergy = totalQuadEnergy + totalEdgeEnergy + disparityEnergy;

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << totalEnergy << ends;

	return totalEnergy;
}

double StereoSolver::stereoEnergy(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight)
{
	double res =  0.0;

	for (unsigned int i = 0; i < originalLeft.vertices.size(); i++)
	{
		Vertex l = originalLeft.vertices.at(i) - newLeft.vertices.at(i);
		Vertex r = originalRight.vertices.at(i) - newRight.vertices.at(i);

		res += sqr(WarpingMath::euclideanNorm(l - r));
	}

	return res;
}