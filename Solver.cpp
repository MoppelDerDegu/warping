#include "Solver.h"
#include "Helper.h"
#include "WarpingMath.h"
#include "lib/nlopt-2.3-dll/nlopt.hpp"


Solver::Solver(Size &originalSize)
{
	this->iterationCount = 0;
	this->oldSize = originalSize;
}


Solver::~Solver(void)
{
}

Mesh Solver::getDeformedMesh()
{
	return deformedMesh;
}

Mesh Solver::getInitialGuess()
{
	return tmp;
}

Mesh Solver::solveImageProblem(Mesh &m, Size &newSize, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving image optimization problem..." << endl;

	this->originalMesh = m;
	this->saliencyWeightMapping = wfMap;
	this->newSize = newSize;

	// initial guess is stored in tmp
	initialGuess(newSize, oldSize);

	// copy initial guess to resultmesh
	deformedMesh = Helper::deepCopyMesh(tmp);
	vector<double> x = Helper::meshToDoubleVec(deformedMesh);

	// formulate optimization problem:

	// derivative free optimization algorithm
	nlopt::opt opt(nlopt::LN_PRAXIS, x.size());

	// lower and upper bounds of vertex coordinates
	vector<double> lb = computeLowerImageBoundConstraints(x, newSize);
	vector<double> ub = computeUpperImageBoundConstraints(x, newSize);
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	// minimize objective function
	opt.set_min_objective(Solver::wrapperImageObjectiveFunc, this);

	// convergence criteria
	opt.set_xtol_abs(1);
	//opt.set_maxtime(60);
	//opt.set_ftol_abs(10);
	//opt.set_maxeval(1000);

	double minf;
	nlopt::result result = opt.optimize(x, minf);
	
	cout << "\n>> Solution found after " << iterationCount << " iterations" << endl;

	iterationCount = 0;
	return deformedMesh;
}

double Solver::wrapperImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	Solver* solv = reinterpret_cast<Solver*> (my_func_data);
	return solv->imageObjFunc(x, grad);
}

// initial guess of the new vertex coordinates, i.e. linear scaling according to the new image size
void Solver::initialGuess(Size &newSize, Size &originalSize)
{
	int newWidth = newSize.width;
	int newHeight = newSize.height;
	int oldHeight = originalSize.height;
	int oldWidth = originalSize.width;

	float scaleX = (float) ((float) newWidth / (float) oldWidth);
	float scaleY = (float) ((float) newHeight / (float) oldHeight);

	tmp = Helper::deepCopyMesh(originalMesh);

	// scale vertices
	for (unsigned int i = 0; i < tmp.vertices.size(); i++)
	{
		tmp.vertices.at(i).x = WarpingMath::round(tmp.vertices.at(i).x * scaleX);
		tmp.vertices.at(i).y = WarpingMath::round(tmp.vertices.at(i).y * scaleY);
	}

	for (unsigned int i = 0; i < tmp.quads.size(); i++)
	{
		tmp.quads.at(i).v1.x = WarpingMath::round(tmp.quads.at(i).v1.x * scaleX);
		tmp.quads.at(i).v1.y = WarpingMath::round(tmp.quads.at(i).v1.y * scaleY);

		tmp.quads.at(i).v2.x = WarpingMath::round(tmp.quads.at(i).v2.x * scaleX);
		tmp.quads.at(i).v2.y = WarpingMath::round(tmp.quads.at(i).v2.y * scaleY);

		tmp.quads.at(i).v3.x = WarpingMath::round(tmp.quads.at(i).v3.x * scaleX);
		tmp.quads.at(i).v3.y = WarpingMath::round(tmp.quads.at(i).v3.y * scaleY);

		tmp.quads.at(i).v4.x = WarpingMath::round(tmp.quads.at(i).v4.x * scaleX);
		tmp.quads.at(i).v4.y = WarpingMath::round(tmp.quads.at(i).v4.y * scaleY);
	}

	for (unsigned int i = 0; i < tmp.edges.size(); i++)
	{
		tmp.edges.at(i).src.x = WarpingMath::round(tmp.edges.at(i).src.x * scaleX);
		tmp.edges.at(i).src.y = WarpingMath::round(tmp.edges.at(i).src.y * scaleY);

		tmp.edges.at(i).dest.x = WarpingMath::round(tmp.edges.at(i).dest.x * scaleX);
		tmp.edges.at(i).dest.y = WarpingMath::round(tmp.edges.at(i).dest.y * scaleY);
	}
}

double Solver::calculateLengthRatio(Edge &oldEdge, Edge &newEdge)
{
	return (WarpingMath::euclideanNorm(newEdge.src - newEdge.dest)) / (WarpingMath::euclideanNorm(oldEdge.src - oldEdge.dest));
}

double Solver::calculateQuadScale(Quad &oldQuad, Quad &newQuad)
{
	double sf;
	double sum1 = 0.0;
	double sum2 = 0.0;

	sum1 += WarpingMath::vTv(oldQuad.v1 - oldQuad.v2, newQuad.v1 - newQuad.v2);
	sum2 += sqr(WarpingMath::euclideanNorm(oldQuad.v1 - oldQuad.v2));

	sum1 += WarpingMath::vTv(oldQuad.v2 - oldQuad.v4, newQuad.v2 - newQuad.v4);
	sum2 += sqr(WarpingMath::euclideanNorm(oldQuad.v2 - oldQuad.v4));

	sum1 += WarpingMath::vTv(oldQuad.v4 - oldQuad.v3, newQuad.v4 - newQuad.v3);
	sum2 += sqr(WarpingMath::euclideanNorm(oldQuad.v4 - oldQuad.v3));

	sum1 += WarpingMath::vTv(oldQuad.v3 - oldQuad.v1, newQuad.v3 - newQuad.v1);
	sum2 += sqr(WarpingMath::euclideanNorm(oldQuad.v3 - oldQuad.v1));

	sf = sum1 / sum2;

	return sf;
}

double Solver::quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf)
{
	double du = 0.0;
	/*
	Vertex _v, v;
	
	_v = newQuad.v1 - newQuad.v2;
	v = oldQuad.v1 - oldQuad.v2;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));

	_v = newQuad.v2 - newQuad.v4;
	v = oldQuad.v2 - oldQuad.v4;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));

	_v = newQuad.v4 - newQuad.v3;
	v = oldQuad.v4 - oldQuad.v3;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));

	_v = newQuad.v3 - newQuad.v1;
	v = oldQuad.v3 - oldQuad.v1;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));
	*/
	
	Vertex a, b, _a, _b;

	a = oldQuad.v1 - oldQuad.v2;
	b = oldQuad.v2 - oldQuad.v4;
	_a = newQuad.v1 - newQuad.v2;
	_b = newQuad.v2 - newQuad.v4;

	double x = sqr(WarpingMath::euclideanNorm(_a - a));
	double y = sqr(WarpingMath::euclideanNorm(_b - b));

	du = x / y;
	
	return du;
}

double Solver::totalQuadEnergy(Mesh &newMesh)
{
	double du = 0.0;

	// assuming #quads in oldmesh == #quads in new mesh
	for (unsigned int i = 0; i < originalMesh.quads.size(); i++)
	{
		// calculate quad scale factor with the initial guess, i.e. sf is constant
		double sf = calculateQuadScale(originalMesh.quads.at(i), tmp.quads.at(i));
		double duf = quadEnergy(originalMesh.quads.at(i), newMesh.quads.at(i), sf);

		// du = du + wf * duf
		du += (saliencyWeightMapping.at(i).first * duf);
	}

	return du;
}

double Solver::totalEdgeEnergy(Mesh &newMesh)
{
	double dl = 0.0;

	for (unsigned int i = 0; i < originalMesh.edges.size(); i++)
	{
		Vertex _v = newMesh.edges.at(i).src - newMesh.edges.at(i).dest;
		Vertex v = originalMesh.edges.at(i).src - originalMesh.edges.at(i).dest;

		// calculate edge lenght ratio
		/*
		double lij = calculateLengthRatio(originalMesh.edges.at(i), tmp.edges.at(i));
		v.x = WarpingMath::round(v.x * lij);
		v.y = WarpingMath::round(v.y * lij);
		*/
		dl += sqr(WarpingMath::euclideanNorm(_v - v));
	}

	return dl;
}

double Solver::imageObjFunc(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	if (!grad.empty())
	{
		// compute gradient here
	}

	Helper::doubleVecToMesh(x, deformedMesh);

	double edgeEnergy = totalEdgeEnergy(deformedMesh);
	double quadEnergy = totalQuadEnergy(deformedMesh);

	double res = edgeEnergy + quadEnergy;

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << res << ends;

	return res;
}

vector<double> Solver::computeLowerImageBoundConstraints(const vector<double> &x, const Size size)
{
	vector<double> lb(x.size());

	for (unsigned int i = 0; i < lb.size(); i++)
	{
		if (i == 0 || i == 1)
		{
			// top left vertex
			lb.at(i) = 0.0;
		}
		else if (i == lb.size() - 2 || i == lb.size() - 1)
		{
			if (i == lb.size() - 2)
			{
				// x-coordinate of bottom right vertex
				lb.at(i) = (double) size.width;
			}
			else
			{
				// y-coordinate of bottom right vertex
				lb.at(i) = (double) size.height;
			}
		}
		else
		{
			if (i % 2 == 0)
			{
				// x-coordinates
				if (((int) x.at(i)) == size.width)
					lb.at(i) = (double) size.width;
				else
					lb.at(i) = 0.0;
			}
			else
			{
				// y-coordinates
				if (((int) x.at(i)) == size.height)
					lb.at(i) = (double) size.height;
				else
					lb.at(i) = 0.0;
			}
		}
	}

	return lb;
}

vector<double> Solver::computeUpperImageBoundConstraints(const vector<double> &x, const Size size)
{
	vector<double> ub(x.size());

	for (unsigned int i = 0; i < ub.size(); i++)
	{
		if (i == 0 || i == 1)
		{
			// top left vertex
			ub.at(i) = 0.0;
		}
		else if (i == ub.size() - 2 || i == ub.size() - 1)
		{
			if (i == ub.size() - 2)
			{
				// x-coordinate of bottom right vertex
				ub.at(i) = (double) size.width;
			}
			else
			{
				// y-coordinate of bottom right vertex
				ub.at(i) = (double) size.height;
			}
		}
		else
		{
			if (i % 2 == 0)
			{
				// x-coordinates
				if (x.at(i) <= 1e-3)
					ub.at(i) = 0.0;
				else
					ub.at(i) = (double) size.width;
			}
			else
			{
				// y-coordinates
				if(x.at(i) <= 1e-3)
					ub.at(i) = 0.0;
				else
					ub.at(i) = (double) size.height;
			}
		}
	}

	return ub;
}

Mesh Solver::redistributeQuads(Mesh &m, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving optimization problem to redistribute quads..." << endl;

	this->deformedMesh = m;
	this->saliencyWeightMapping = wfMap;

	vector<double> x = Helper::meshToDoubleVec(deformedMesh);

	vector<double> lb = computeLowerImageBoundConstraints(x, oldSize);
	vector<double> ub = computeUpperImageBoundConstraints(x, oldSize);

	nlopt::opt opt(nlopt::LN_PRAXIS, x.size());

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	opt.set_min_objective(Solver::wrapperRedistributeObjectiveFunc, this);

	opt.set_xtol_abs(1);

	double minf;
	nlopt::result res = opt.optimize(x, minf);
	
	cout << "\n>> Solution found after " << iterationCount << " iterations" << endl;

	iterationCount = 0;
	return deformedMesh;
}

double Solver::wrapperRedistributeObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
{
	Solver* solver = reinterpret_cast<Solver*> (my_func_data);
	return solver->redistributeObjFunc(x, grad);
}

double Solver::redistributeObjFunc(const vector<double> &x, vector<double> &grad)
{
	++iterationCount;

	double res;

	if (!grad.empty())
	{
		// compute gradient here
	}

	Helper::doubleVecToMesh(x, deformedMesh);
	res = totalRedistributionEnergy(deformedMesh);

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << res << ends;

	return res;
}

double Solver::totalRedistributionEnergy(Mesh &newMesh)
{
	double sum = 0;
	
	for (unsigned int i = 0; i < newMesh.edges.size(); i++)
	{
		sum += ((1 + edgeSaliency(newMesh, newMesh.edges.at(i))) * sqr(WarpingMath::euclideanNorm(newMesh.edges.at(i).src - newMesh.edges.at(i).dest)));
	}

	return sum;
}

double Solver::edgeSaliency(Mesh &m, Edge &e)
{
	if (isEdgeOnBorder(e, oldSize))
	{

	}
	else
	{

	}

	return 0.0;
}

bool Solver::isEdgeOnBorder(Edge &e, Size &size)
{
	return (e.src.x == 0 && e.dest.x == 0) || (e.src.y == 0 && e.dest.y == 0) || (e.src.x == size.width && e.dest.x == size.width) || (e.src.y == size.height && e.dest.y == size.height);
}