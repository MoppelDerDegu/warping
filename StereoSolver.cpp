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
	cout << "> Solving stereo image optimization problem..." << endl;

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
	opt.set_xtol_abs(10);

	// set boundary constraints
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	double minf;

	for (int i = 0; i < 20; i++)
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
	/*
	double edgeEnergyLeft = totalEdgeEnergy(originalLeft, deformedLeft, edgeLengthRatiosLeft);
	double edgeEnergyRight = totalEdgeEnergy(originalRight, deformedRight, edgeLengthRatiosRight);

	double totalEdgeEnergy = edgeEnergyLeft + edgeEnergyRight;
	*/
	double disparityEnergy = stereoEnergy(originalLeft, originalRight, deformedLeft, deformedRight);

	double totalEnergy = totalQuadEnergy + /*totalEdgeEnergy +*/ disparityEnergy;

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

vector<double> StereoSolver::computeLowerXBoundConstraints(const vector<double> &x, const Size size)
{
	vector<double> lb(x.size());

	for (unsigned int i = 0; i < lb.size(); i++)
	{
		if (i == 0)
		{
			// x-coordinate of top left vertex
			lb.at(i) = 0.0;
		}
		else if (i == lb.size() - 1)
		{
			// x-coordinate of bottom right vertex
			lb.at(i) = (double) size.width;
		}
		else
		{
			if (((int) x.at(i)) == size.width)
				lb.at(i) = (double) size.width;
			else
				lb.at(i) = 0.0;	
		}
	}

	return lb;
}

vector<double> StereoSolver::computeUpperXBoundConstraints(const vector<double> &x, const Size size)
{
	vector<double> ub(x.size());

	for (unsigned int i = 0; i < ub.size(); i++)
	{
		if (i == 0)
		{
			// x-coordinate of top left vertex
			ub.at(i) = 0.0;
		}
		else if (i == ub.size() - 1)
		{
			// x-coordinate of bottom right vertex
			ub.at(i) = (double) size.width;
		}
		else
		{
			if (x.at(i) <= 1e-3)
				ub.at(i) = 0.0;
			else
				ub.at(i) = (double) size.width;
		}
	}

	return ub;
}

vector<double> StereoSolver::computeLowerYBoundConstraints(const vector<double> &x, const Size size)
{
	vector<double> lb(x.size());

	for (unsigned int i = 0; i < lb.size(); i++)
	{
		if (i == 0)
		{
			// y-coordinate of top left vertex
			lb.at(i) = 0.0;
		}
		else if (i == lb.size() - 1)
		{
			// y-coordinate of bottom right vertex
			lb.at(i) = (double) size.height;
		}
		else
		{
			if (((int) x.at(i)) == size.height)
				lb.at(i) = (double) size.height;
			else
				lb.at(i) = 0.0;	
		}
	}

	return lb;
}

vector<double> StereoSolver::computeUpperYBoundConstraints(const vector<double> &x, const Size size)
{
	vector<double> ub(x.size());

	for (unsigned int i = 0; i < ub.size(); i++)
	{
		if (i == 0)
		{
			// y-coordinate of top left vertex
			ub.at(i) = 0.0;
		}
		else if (i == ub.size() - 1)
		{
			// y-coordinate of bottom right vertex
			ub.at(i) = (double) size.height;
		}
		else
		{
			if(x.at(i) <= 1e-3)
				ub.at(i) = 0.0;
			else
				ub.at(i) = (double) size.height;
		}
	}

	return ub;
}

double StereoSolver::totalQuadEnergyX(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Quad, float>> &scalingFactors, const vector<pair<float, Quad>> &saliencyWeightMapping)
{
	double dux = 0.0;

	// assuming #quads in oldmesh == #quads in new mesh
	for (unsigned int i = 0; i < originalMesh.quads.size(); i++)
	{
		double sf = scalingFactors.at(i).second;
		double dufx = quadEnergyX(originalMesh.quads.at(i), newMesh.quads.at(i), sf);

		// dux = dux + wf * dufx
		dux += ((saliencyWeightMapping.at(i).first) * dufx);
	}

	return dux;
}

double StereoSolver::totalQuadEnergyY(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Quad, float>> &scalingFactors, const vector<pair<float, Quad>> &saliencyWeightMapping)
{
	double duy = 0.0;

	// assuming #quads in oldmesh == #quads in new mesh
	for (unsigned int i = 0; i < originalMesh.quads.size(); i++)
	{
		double sf = scalingFactors.at(i).second;
		double dufy = quadEnergyY(originalMesh.quads.at(i), newMesh.quads.at(i), sf);

		// dux = dux + wf * dufx
		duy += ((saliencyWeightMapping.at(i).first) * dufy);
	}

	return duy;
}

double StereoSolver::quadEnergyX(Quad &oldQuad, Quad &newQuad, const double sf)
{
	double du = 0.0;

	double x, _x;

	_x = newQuad.v1.x - newQuad.v2.x;
	x = (oldQuad.v1.x - oldQuad.v2.x) * sf;

	du += sqr(_x - x);

	_x = newQuad.v2.x - newQuad.v4.x;
	x = (oldQuad.v2.x - oldQuad.v4.x) * sf;

	du += sqr(_x - x);

	_x = newQuad.v4.x - newQuad.v3.x;
	x = (oldQuad.v4.x - oldQuad.v3.x) * sf;

	du += sqr(_x - x);

	_x = newQuad.v3.x - newQuad.v1.x;
	x = (oldQuad.v3.x - oldQuad.v1.x) * sf;

	du += sqr(_x - x);

	return du;
}

double StereoSolver::quadEnergyY(Quad &oldQuad, Quad &newQuad, const double sf)
{
	double du = 0.0;

	double y, _y;

	_y = newQuad.v1.y - newQuad.v2.y;
	y = (oldQuad.v1.y - oldQuad.v2.y) * sf;

	du += sqr(_y - y);

	_y = newQuad.v2.y - newQuad.v4.y;
	y = (oldQuad.v2.y - oldQuad.v4.y) * sf;

	du += sqr(_y - y);

	_y = newQuad.v4.y - newQuad.v3.y;
	y = (oldQuad.v4.y - oldQuad.v3.y) * sf;

	du += sqr(_y - y);

	_y = newQuad.v3.y - newQuad.v1.y;
	y = (oldQuad.v3.y - oldQuad.v1.y) * sf;

	du += sqr(_y - y);

	return du;
}

double StereoSolver::stereoEnergyX(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight)
{
	double dx = 0.0;

	for (unsigned int i = 0; i < originalLeft.vertices.size(); i++)
	{
		dx += sqr((originalLeft.vertices.at(i).x - newLeft.vertices.at(i).x) - (originalRight.vertices.at(i).x - newRight.vertices.at(i).x));
	}

	return dx;
}

double StereoSolver::stereoEnergyY(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight)
{
	double dy = 0.0;

	for (unsigned int i = 0; i < originalLeft.vertices.size(); i++)
	{
		dy += sqr((originalLeft.vertices.at(i).y - newLeft.vertices.at(i).y) - (originalRight.vertices.at(i).y - newRight.vertices.at(i).y));
	}

	return dy;
}

double StereoSolver::stereoImageObjFuncX(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	if (!grad.empty())
	{
		// compute gradient here
	}

	MeshManager* mm = MeshManager::getInstance();

	// split array into left and right mesh
	vector<double> left(x.begin(), x.begin() + splitIndexSeperate);
	vector<double> right(x.begin() + splitIndexSeperate, x.end());

	mm->xCoordsToMesh(left, deformedLeftXOnly);
	mm->xCoordsToMesh(right, deformedRightXOnly);
	
	double quadEnergyLeft = totalQuadEnergyX(originalLeft, deformedLeftXOnly, scalingFactorsLeft, saliencyWeightMappingLeft);
	double quadEnergyRight = totalQuadEnergyX(originalRight, deformedRightXOnly, scalingFactorsRight, saliencyWeightMappingRight);

	double totalQuadEnergy = quadEnergyLeft + quadEnergyRight;

	double disparityEnergy = stereoEnergyX(originalLeft, originalRight, deformedLeftXOnly, deformedRightXOnly);
	
	double totalEnergy = totalQuadEnergy + /*totalEdgeEnergy +*/ disparityEnergy;

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << totalEnergy << ends;

	return totalEnergy;
}

double StereoSolver::stereoImageObjFuncY(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	if (!grad.empty())
	{
		// compute gradient here
	}

	MeshManager* mm = MeshManager::getInstance();

	// split array into left and right mesh
	vector<double> left(x.begin(), x.begin() + splitIndexSeperate);
	vector<double> right(x.begin() + splitIndexSeperate, x.end());

	mm->yCoordsToMesh(left, deformedLeftYOnly);
	mm->yCoordsToMesh(right, deformedRightYOnly);
	
	double quadEnergyLeft = totalQuadEnergyY(originalLeft, deformedLeftYOnly, scalingFactorsLeft, saliencyWeightMappingLeft);
	double quadEnergyRight = totalQuadEnergyY(originalRight, deformedRightYOnly, scalingFactorsRight, saliencyWeightMappingRight);

	double totalQuadEnergy = quadEnergyLeft + quadEnergyRight;

	double disparityEnergy = stereoEnergyY(originalLeft, originalRight, deformedLeftYOnly, deformedRightYOnly);
	
	double totalEnergy = totalQuadEnergy + /*totalEdgeEnergy +*/ disparityEnergy;

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << totalEnergy << ends;

	return totalEnergy;
}

double StereoSolver::wrapperStereoImageObjectiveX(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	StereoSolver* solver = reinterpret_cast<StereoSolver*> (my_func_data);
	return solver->stereoImageObjFuncX(x, grad);
}

double StereoSolver::wrapperStereoImageObjectiveY(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	StereoSolver* solver = reinterpret_cast<StereoSolver*> (my_func_data);
	return solver->stereoImageObjFuncY(x, grad);
}

pair<Mesh, Mesh> StereoSolver::solveStereoImageProblemSeperately(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight)
{
	cout << "> Solving stereo image optimization problem..." << endl;

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

	deformedLeftXOnly = mm->deepCopyMesh(initialLeft);
	deformedLeftYOnly = mm->deepCopyMesh(initialLeft);
	deformedRightXOnly = mm->deepCopyMesh(initialRight);
	deformedRightYOnly = mm->deepCopyMesh(initialRight);

	// left
	vector<double> xLeft = mm->xCoordsToDoubleVec(deformedLeftXOnly);
	vector<double> yLeft = mm->yCoordsToDoubleVec(deformedLeftYOnly);

	// compute image boundaries x
	vector<double> lbx = computeLowerXBoundConstraints(xLeft, newLeftSize);
	lbx.insert(lbx.end(), lbx.begin(), lbx.end());
	vector<double> ubx = computeUpperXBoundConstraints(xLeft, newLeftSize);
	ubx.insert(ubx.end(), ubx.begin(), ubx.end());

	// comput image boundaries y
	vector<double> lby = computeLowerYBoundConstraints(yLeft, newLeftSize);
	lby.insert(lby.end(), lby.begin(), lby.end());
	vector<double> uby = computeUpperYBoundConstraints(yLeft, newLeftSize);
	uby.insert(uby.end(), uby.begin(), uby.end());

	// right
	vector<double> xRight = mm->xCoordsToDoubleVec(deformedRightXOnly);
	vector<double> yRight = mm->yCoordsToDoubleVec(deformedRightYOnly);
	splitIndexSeperate = xRight.size();
	
	// append the right mesh to the vector containing the left mesh for x and y coordinates respectively
	xLeft.insert(xLeft.end(), xRight.begin(), xRight.end());
	yLeft.insert(yLeft.end(), yRight.begin(), yRight.end());

	// formulate optimization problem for x and y coordinates:

	// derivative free optimization algorithm
	nlopt::opt optX(nlopt::LN_NELDERMEAD, xLeft.size());
	nlopt::opt optY(nlopt::LN_NELDERMEAD, yLeft.size());

	// minimize objective function
	optX.set_min_objective(StereoSolver::wrapperStereoImageObjectiveX, this);
	optY.set_min_objective(StereoSolver::wrapperStereoImageObjectiveX, this);

	// convergence criteria
	optX.set_xtol_abs(10);
	optY.set_xtol_abs(10);

	// set boundary constraints
	optX.set_lower_bounds(lbx);
	optX.set_upper_bounds(ubx);
	optY.set_lower_bounds(lby);
	optY.set_upper_bounds(uby);

	double minfX, minfY;

	for (int i = 0 ; i < 1; i++)
	{	
		calculateEdgeLengthRatios(originalLeft, deformedLeft, edgeLengthRatiosLeft);
		calculateEdgeLengthRatios(originalRight, deformedRight, edgeLengthRatiosRight);

		calculateOptimalScaleFactors(originalLeft, deformedLeft, scalingFactorsLeft);
		calculateOptimalScaleFactors(originalRight, deformedRight, scalingFactorsRight);
			
		cout << ">> Solving for x coordinates for step " << i + 1 << endl;
		nlopt::result resultX = optX.optimize(xLeft, minfX);
		cout << "\n>>> Solution found after " << iterationCount << " iterations" << endl;
		iterationCount = 0;

		cout << ">> Solving for y coordinates for step " << i + 1 << endl;
		nlopt::result resultY = optY.optimize(yLeft, minfY);
		cout << "\n>>> Solution found after " << iterationCount << " iterations" << endl;
		iterationCount = 0;

		// merge x and y coordinates of left mesh
		mm->mergeXandYMeshes(deformedLeftXOnly, deformedLeftYOnly, deformedLeft);

		// merge x and y coordinates of right mesh
		mm->mergeXandYMeshes(deformedRightXOnly, deformedRightYOnly, deformedRight);
	}

	FileManager::saveMeshAsImage("deformed left x coords.png", "D:\\warping\\mesh\\", deformedLeftXOnly, newSize);
	FileManager::saveMeshAsImage("deformed left y coords.png", "D:\\warping\\mesh\\", deformedLeftYOnly, newSize);
	
	FileManager::saveMeshAsImage("deformed right x coords.png", "D:\\warping\\mesh\\", deformedRightXOnly, newSize);
	FileManager::saveMeshAsImage("deformed right y coords.png", "D:\\warping\\mesh\\", deformedRightYOnly, newSize);

	pair<Mesh, Mesh> p(deformedLeft, deformedRight);

	return p;
}