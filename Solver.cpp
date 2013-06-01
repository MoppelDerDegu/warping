#include "Solver.h"
#include "Helper.h"
#include "QuadSaliencyManager.h"
#include "WarpingMath.h"
#include "FileManager.h"
#include "lib/nlopt-2.3-dll/nlopt.hpp"


Solver::Solver(Size &originalSize)
{
	this->iterationCount = 0;
	this->quadscale = 0.0;
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

Mesh Solver::solveImageProblem(Mesh &contentAwareMesh, Mesh &originalMesh, Size &newSize, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving image optimization problem..." << endl;

	this->originalMesh = originalMesh;
	this->contentAwareMesh = contentAwareMesh;
	this->saliencyWeightMapping = wfMap;
	this->newSize = newSize;

	// initial guess is stored in tmp
	initialGuess(newSize, oldSize);
	
	// copy initial guess to resultmesh
	deformedMesh = Helper::deepCopyMesh(tmp);
	vector<double> x = Helper::meshToDoubleVec(deformedMesh);

	QuadSaliencyManager qsm;
	edgeSaliency = qsm.assignSaliencyValuesToEdges(contentAwareMesh, saliencyWeightMapping, oldSize);

	// formulate optimization problem:

	// derivative free optimization algorithm
	nlopt::opt opt(nlopt::LN_NELDERMEAD, x.size());

	// lower and upper bounds of vertex coordinates
	vector<double> lb = computeLowerImageBoundConstraints(x, newSize);
	vector<double> ub = computeUpperImageBoundConstraints(x, newSize);
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	// minimize objective function
	opt.set_min_objective(Solver::wrapperImageObjectiveFunc, this);

	// convergence criteria
	opt.set_xtol_abs(1);

	double minf;

	//cout << "\n>>Solving problem for step " << i << endl;

	//calculateEdgeLengthRatios();
	//calculateOptimalScaleFactors();
	calculateQuadScale();
	edgeSaliency = qsm.assignSaliencyValuesToEdges(contentAwareMesh, saliencyWeightMapping, oldSize);
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

	float scaleX = (float) newWidth / (float) oldWidth;
	float scaleY = (float) newHeight / (float) oldHeight;

	tmp = Helper::deepCopyMesh(contentAwareMesh);

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
	
	/*
	sum1 += WarpingMath::vTv(oldQuad.v1, newQuad.v1);
	sum2 += WarpingMath::vTv(oldQuad.v1, oldQuad.v1);

	sum1 += WarpingMath::vTv(oldQuad.v2, newQuad.v2);
	sum2 += WarpingMath::vTv(oldQuad.v2, oldQuad.v2);

	sum1 += WarpingMath::vTv(oldQuad.v3, newQuad.v3);
	sum2 += WarpingMath::vTv(oldQuad.v3, oldQuad.v3);
	
	sum1 += WarpingMath::vTv(oldQuad.v4, newQuad.v4);
	sum2 += WarpingMath::vTv(oldQuad.v4, oldQuad.v4);

	sf = sum1 / sum2;
	*/
	return sf;
}

double Solver::quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf)
{
	double du = 0.0;
	
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
	
	/*
	Vertex v, _v;

	v = oldQuad.v1;
	_v = newQuad.v1;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));

	v = oldQuad.v2;
	_v = newQuad.v2;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));

	v = oldQuad.v3;
	_v = newQuad.v3;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));

	v = oldQuad.v4;
	_v = newQuad.v4;
	v.x = WarpingMath::round(v.x * sf);
	v.y = WarpingMath::round(v.y * sf);

	du += sqr(WarpingMath::euclideanNorm(_v - v));
	*/
	return du;
}

double Solver::totalQuadEnergy(Mesh &newMesh)
{
	double du = 0.0;

	// assuming #quads in oldmesh == #quads in new mesh
	for (unsigned int i = 0; i < contentAwareMesh.quads.size(); i++)
	{
		double sf = quadscale;
		double duf = quadEnergy(contentAwareMesh.quads.at(i), newMesh.quads.at(i), sf);

		// du = du + wf * duf
		du += ((0.05*saliencyWeightMapping.at(i).first) * duf);
	}

	return du;
}

double Solver::totalEdgeEnergy(Mesh &newMesh)
{
	double dl = 0.0;

	for (unsigned int i = 0; i < contentAwareMesh.edges.size(); i++)
	{	
		Vertex _v = newMesh.edges.at(i).src - newMesh.edges.at(i).dest;
		Vertex v = contentAwareMesh.edges.at(i).src - contentAwareMesh.edges.at(i).dest;
		
		double lij = calculateLengthRatio(contentAwareMesh.edges.at(i), tmp.edges.at(i));
		v.x = WarpingMath::round(v.x * lij);
		v.y = WarpingMath::round(v.y * lij);
		
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
	
	QuadSaliencyManager qsm;
	this->edgeSaliency = qsm.assignSaliencyValuesToEdges(m, saliencyWeightMapping, oldSize);

	vector<double> x = Helper::meshToDoubleVec(deformedMesh);

	vector<double> lb = computeLowerImageBoundConstraints(x, oldSize);
	vector<double> ub = computeUpperImageBoundConstraints(x, oldSize);

	nlopt::opt opt(nlopt::LN_PRAXIS, x.size());

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	opt.set_min_objective(Solver::wrapperRedistributeObjectiveFunc, this);

	opt.set_xtol_abs(1);
	//opt.set_maxeval(1000);

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

	if (!grad.empty())
	{
		// compute gradient here
	}

	Helper::doubleVecToMesh(x, deformedMesh);
	double res = totalRedistributionEnergy(deformedMesh);

	cout << "\r>> Iteration: " << iterationCount << " Total Energy: " << res << ends;

	return res;
}

double Solver::totalRedistributionEnergy(Mesh &newMesh)
{
	double sum = 0;
	
	for (unsigned int i = 0; i < newMesh.edges.size(); i++)
	{
		Edge e = newMesh.edges.at(i);
		sum += ((1 + edgeSaliency.at(i).second) * sqr(WarpingMath::euclideanNorm(newMesh.edges.at(i).src - newMesh.edges.at(i).dest)));
	}
	
	return sum;
}

void Solver::calculateEdgeLengthRatios()
{
	edgeLengthRatios.clear();

	for (unsigned int i = 0; i < deformedMesh.edges.size(); i++)
	{
		pair<Edge, float> p;
		p.first = deformedMesh.edges.at(i);
		p.second = calculateLengthRatio(contentAwareMesh.edges.at(i), deformedMesh.edges.at(i));

		edgeLengthRatios.push_back(p);
	}
}

void Solver::calculateOptimalScaleFactors()
{
	scalingFactors.clear();

	for (unsigned int i = 0; i < deformedMesh.quads.size(); i++)
	{
		pair<Quad, float> p;
		p.first = deformedMesh.quads.at(i);
		p.second = calculateQuadScale(contentAwareMesh.quads.at(i), deformedMesh.quads.at(i));

		scalingFactors.push_back(p);
	}
}

void Solver::calculateQuadScale()
{
	bool bigger = false;

	int oldArea = oldSize.width * oldSize.height;
	int newArea = newSize.width * newSize.height;

	if (oldArea < newArea)
		bigger = true;
	
	if (bigger)
		quadscale = max(newSize.height / (float) oldSize.height, newSize.width / (float) oldSize.width);
	else
		quadscale = min(newSize.height / (float) oldSize.height, newSize.width / (float) oldSize.width);
}