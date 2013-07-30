#include "StereoPathlineSolver.h"
#include "PathlineManager.h"
#include "MeshManager.h"
#include "WarpingMath.h"
#include "lib\nlopt-2.3-dll\nlopt.hpp"

StereoPathlineSolver::StereoPathlineSolver(unsigned int maxEval, int frame, PathlineSets &leftOptimizedPathlines, PathlineSets &rightOptimizedPathlines, PathlineSets &leftOriginalPathlines, PathlineSets &rightOriginalPathlines)
{
	iterationCount = 0;
	this->maxEval = maxEval;
	this->frame = frame;
	this->leftOptimizedPathlines = leftOptimizedPathlines;
	this->rightOptimizedPathlines = rightOptimizedPathlines;
	this->leftOriginalPathlines = leftOriginalPathlines;
	this->rightOriginalPathlines = rightOriginalPathlines;
}

StereoPathlineSolver::~StereoPathlineSolver(void)
{
}

pair<Mesh, Mesh> StereoPathlineSolver::solveStereoImageProblem(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight)
{
	cout << "> Solving stereo image optimization problem regarding pathlines..." << endl;

	MeshManager* mm = MeshManager::getInstance();
	PathlineManager* pm = PathlineManager::getInstance();

	// initialization
	this->saliencyWeightMappingLeft = wfMapLeft;
	this->saliencyWeightMappingRight = wfMapRight;
	this->originalLeft = originalLeft;
	this->originalRight = originalRight;

	// map pathlines to quads
	pm->mapPathlinesToQuads(frame, leftOriginalPathlines, originalLeft, leftQuadMapping);
	pm->mapPathlinesToQuads(frame, rightOriginalPathlines, originalRight, rightQuadMapping);

	// get set of pathlines passing this' frame
	pm->getLinesContainingFrame(leftOriginalPathlines, frame, leftOriginalLines);
	pm->getLinesContainingFrame(rightOriginalPathlines, frame, rightOriginalLines);
	pm->getLinesContainingFrame(leftOptimizedPathlines, frame, leftOptimizedLines);
	pm->getLinesContainingFrame(rightOptimizedPathlines, frame, rightOptimizedLines);

	// determine all pathline points in this' frame
	pm->getPointsInFrame(leftOriginalLines, frame, leftOriginalPathlinePoints);
	pm->getPointsInFrame(rightOriginalLines, frame, rightOriginalPathlinePoints);
	pm->getPointsInFrame(leftOptimizedLines, frame, leftOptimizedPathlinePoints);
	pm->getPointsInFrame(rightOptimizedLines, frame, rightOptimizedPathlinePoints);

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
	nlopt::opt opt(nlopt::LN_NELDERMEAD, x.size());

	// minimize objective function
	opt.set_min_objective(StereoPathlineSolver::wrapperObjFunc, this);

	// convergence criteria
	opt.set_xtol_abs(5);

	// set boundary constraints
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);
	
	double minf;

	for (int i = 0; i < maxEval; i++)
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

double StereoPathlineSolver::wrapperObjFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	StereoPathlineSolver* sps = reinterpret_cast<StereoPathlineSolver*> (my_func_data);
	return sps->objFunc(x, grad);
}

double StereoPathlineSolver::objFunc(const vector<double> &x, vector<double> &grad)
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
	/*
	double edgeEnergyLeft = totalEdgeEnergy(originalLeft, deformedLeft, edgeLengthRatiosLeft);
	double edgeEnergyRight = totalEdgeEnergy(originalRight, deformedRight, edgeLengthRatiosRight);

	double totalEdgeEnergy = edgeEnergyLeft + edgeEnergyRight;
	*/
	double disparityEnergy = stereoEnergy(originalLeft, originalRight, deformedLeft, deformedRight);

	double pathlineEnergyLeft = pathlineEnergy(deformedLeft, leftOptimizedPathlinePoints, true);
	double pathlineEnergyRight = pathlineEnergy(deformedRight, rightOptimizedPathlinePoints, false);

	double totalPathlineEnergy = pathlineEnergyLeft + pathlineEnergyRight;

	double totalEnergy = totalQuadEnergy + /*totalEdgeEnergy +*/ disparityEnergy + totalPathlineEnergy;

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << totalEnergy << ends;

	return totalEnergy;
}

double StereoPathlineSolver::pathlineEnergy(Mesh &mesh, vector<Point2f> &guidePoints, bool left)
{
	double res = 0;

	for (unsigned int i = 0; i < guidePoints.size(); i++)
	{
		Quad q;
		
		if (left)
			q = mesh.quads.at(leftQuadMapping.at(i));
		else
			q = mesh.quads.at(rightQuadMapping.at(i));

		Vertex mean = (q.v1 + q.v2 + q.v3 + q.v4) * 0.25;
		Point2f p(mean.x, mean.y);

		res += sqr(WarpingMath::euclideanNorm(p - guidePoints.at(i)));
	}

	return res;
}