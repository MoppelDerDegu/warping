#include "Solver.h"
#include "Helper.h"
#include "lib/nlopt-2.3-dll/nlopt.hpp"


Solver::Solver(void)
{
	iterationCount = 0;
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

Mesh Solver::solveImageProblem(Mesh &m, Size &newSize, Size &originalSize, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving image optimization problem..." << endl;

	this->originalMesh = m;
	this->saliencyWeightMapping = wfMap;
	this->oldSize = originalSize;
	this->newSize = newSize;

	// initial guess is stored in tmp
	initialGuess(newSize, originalSize);

	// copy initial guess to resultmesh
	deformedMesh = Helper::deepCopyMesh(tmp);
	vector<double> x = Helper::meshToDoubleVec(deformedMesh);

	// formulate optimization problem:

	// derivative free optimization algorithm
	nlopt::opt opt(nlopt::LN_SBPLX, x.size());

	// lower and upper bounds of vertex coordinates
	vector<double> lb = computeLowerImageBoundConstraints(x);
	vector<double> ub = computeUpperImageBoundConstraints(x);
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	// minimize objective function
	opt.set_min_objective(Solver::wrapperOptFunc, this);

	// convergence criteria
	opt.set_xtol_abs(10);
	//opt.set_maxtime(60);
	//opt.set_ftol_abs(10);

	double minf;
	//nlopt::result result = opt.optimize(x, minf);
	
	cout << "\n>> Solution found after " << iterationCount << " iterations" << endl;

	return deformedMesh;
}

double Solver::wrapperOptFunc(const vector<double> &x, vector<double> &grad, void *my_func_data)
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
		tmp.vertices.at(i).x = Helper::round(tmp.vertices.at(i).x * scaleX);
		tmp.vertices.at(i).y = Helper::round(tmp.vertices.at(i).y * scaleY);
	}

	for (unsigned int i = 0; i < tmp.quads.size(); i++)
	{
		tmp.quads.at(i).v1.x = Helper::round(tmp.quads.at(i).v1.x * scaleX);
		tmp.quads.at(i).v1.y = Helper::round(tmp.quads.at(i).v1.y * scaleY);

		tmp.quads.at(i).v2.x = Helper::round(tmp.quads.at(i).v2.x * scaleX);
		tmp.quads.at(i).v2.y = Helper::round(tmp.quads.at(i).v2.y * scaleY);

		tmp.quads.at(i).v3.x = Helper::round(tmp.quads.at(i).v3.x * scaleX);
		tmp.quads.at(i).v3.y = Helper::round(tmp.quads.at(i).v3.y * scaleY);

		tmp.quads.at(i).v4.x = Helper::round(tmp.quads.at(i).v4.x * scaleX);
		tmp.quads.at(i).v4.y = Helper::round(tmp.quads.at(i).v4.y * scaleY);
	}

	for (unsigned int i = 0; i < tmp.edges.size(); i++)
	{
		tmp.edges.at(i).src.x = Helper::round(tmp.edges.at(i).src.x * scaleX);
		tmp.edges.at(i).src.y = Helper::round(tmp.edges.at(i).src.y * scaleY);

		tmp.edges.at(i).dest.x = Helper::round(tmp.edges.at(i).dest.x * scaleX);
		tmp.edges.at(i).dest.y = Helper::round(tmp.edges.at(i).dest.y * scaleY);
	}
}

double Solver::calculateLengthRatio(Edge &oldEdge, Edge &newEdge)
{
	return (Helper::euclideanNorm(newEdge.src - newEdge.dest)) / (Helper::euclideanNorm(oldEdge.src - oldEdge.dest));
}

double Solver::calculateQuadScale(Quad &oldQuad, Quad &newQuad)
{
	double sf;
	double sum1 = 0.0;
	double sum2 = 0.0;

	for (int i = 0; i < 4; i++)
	{
		if (i == 0)
		{
			sum1 += vTv(oldQuad.v1 - oldQuad.v2, newQuad.v1 - newQuad.v2);
			sum2 += sqr(Helper::euclideanNorm(oldQuad.v1 - oldQuad.v2));
		}
		else if (i == 1)
		{
			sum1 += vTv(oldQuad.v2 - oldQuad.v4, newQuad.v2 - newQuad.v4);
			sum2 += sqr(Helper::euclideanNorm(oldQuad.v2 - oldQuad.v4));
		}
		else if (i == 2)
		{
			sum1 += vTv(oldQuad.v4 - oldQuad.v3, newQuad.v4 - newQuad.v3);
			sum2 += sqr(Helper::euclideanNorm(oldQuad.v4 - oldQuad.v3));
		}
		else
		{
			sum1 += vTv(oldQuad.v3 - oldQuad.v1, newQuad.v3 - newQuad.v1);
			sum2 += sqr(Helper::euclideanNorm(oldQuad.v3 - oldQuad.v1));
		}
	}

	sf = sum1 / sum2;

	return sf;
}

double Solver::quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf)
{
	double du = 0.0;

	for (int i = 0; i < 4; i++)
	{
		if (i == 0)
		{
			Vertex _v = newQuad.v1 - newQuad.v2;
			Vertex v = oldQuad.v1 - oldQuad.v2;
			v.x = Helper::round(v.x * sf);
			v.y = Helper::round(v.y * sf);

			du += sqr(Helper::euclideanNorm(_v - v));
		}
		else if (i == 1)
		{
			Vertex _v = newQuad.v2 - newQuad.v4;
			Vertex v = oldQuad.v2 - oldQuad.v4;
			v.x = Helper::round(v.x * sf);
			v.y = Helper::round(v.y * sf);

			du += sqr(Helper::euclideanNorm(_v - v));
		}
		else if (i == 2)
		{
			Vertex _v = newQuad.v4 - newQuad.v3;
			Vertex v = oldQuad.v4 - oldQuad.v3;
			v.x = Helper::round(v.x * sf);
			v.y = Helper::round(v.y * sf);

			du += sqr(Helper::euclideanNorm(_v - v));
		}
		else
		{
			Vertex _v = newQuad.v3 - newQuad.v1;
			Vertex v = oldQuad.v3 - oldQuad.v1;
			v.x = Helper::round(v.x * sf);
			v.y = Helper::round(v.y * sf);

			du += sqr(Helper::euclideanNorm(_v - v));
		}
	}

	return du;
}

double Solver::totalQuadEnergy(Mesh &newMesh)
{
	double du = 0.0;

	// assuming #quads in oldmesh == #quads in new mesh
	for (unsigned int i = 0; i < tmp.quads.size(); i++)
	{
		// calculate quad scale factor with the initial guess, i.e. sf is constant
		//double sf = calculateQuadScale(tmp.quads.at(i), newMesh.quads.at(i));
		double duf = quadEnergy(tmp.quads.at(i), newMesh.quads.at(i), 0.0);

		// du = du + wf * duf
		du += (saliencyWeightMapping.at(i).first * duf);
	}

	return du;
}

double Solver::totalEdgeEnergy(Mesh &newMesh)
{
	double dl = 0.0;

	for (unsigned int i = 0; i < tmp.edges.size(); i++)
	{
		Vertex _v = newMesh.edges.at(i).src - newMesh.edges.at(i).dest;
		Vertex v = tmp.edges.at(i).src - tmp.edges.at(i).dest;

		// calculate edge lenght ratio
		double lij = calculateLengthRatio(tmp.edges.at(i), newMesh.edges.at(i));
		v.x = Helper::round(v.x * lij);
		v.y = Helper::round(v.y * lij);

		dl += sqr(Helper::euclideanNorm(_v/* - v*/));
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

double Solver::vTv(Vertex v1, Vertex v2)
{
	return (double) (v1.x * v2.x + v1.y * v2.y);
}

vector<double> Solver::computeLowerImageBoundConstraints(const vector<double> &x)
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
				lb.at(i) = (double) newSize.width;
			}
			else
			{
				// y-coordinate of bottom right vertex
				lb.at(i) = (double) newSize.height;
			}
		}
		else
		{
			if (i % 2 == 0)
			{
				// x-coordinates
				if (((int) x.at(i)) == newSize.width)
					lb.at(i) = (double) newSize.width;
				else
					lb.at(i) = 0.0;
			}
			else
			{
				// y-coordinates
				if (((int) x.at(i)) == newSize.height)
					lb.at(i) = (double) newSize.height;
				else
					lb.at(i) = 0.0;
			}
		}
	}

	return lb;
}

vector<double> Solver::computeUpperImageBoundConstraints(const vector<double> &x)
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
				ub.at(i) = (double) newSize.width;
			}
			else
			{
				// y-coordinate of bottom right vertex
				ub.at(i) = (double) newSize.height;
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
					ub.at(i) = (double) newSize.width;
			}
			else
			{
				// y-coordinates
				if(x.at(i) <= 1e-3)
					ub.at(i) = 0.0;
				else
					ub.at(i) = (double) newSize.height;
			}
		}
	}

	return ub;
}