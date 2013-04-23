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

Mesh Solver::solveImageProblem(Mesh &m, Size &newSize, Size &originalSize, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving image optimization problem..." << endl;

	this->originalMesh = m;
	this->saliencyWeightMapping = wfMap;
	this->oldSize = originalSize;
	this->newSize = newSize;
	this->originalQuadWidth = m.vertices.at(1).x;
	this->originalQuadHeight = m.vertices.at(2).y;

	// initial guess is stored in tmp
	initialGuess(newSize, originalSize);
	// copy initial guess to resultmesh
	deformedMesh = Helper::deepCopyMesh(tmp);
	vector<double> x = meshToDoubleVec(deformedMesh);

	// formulate optimization problem:

	// derivative free optimization algorithm COBYLA
	nlopt::opt opt(nlopt::LN_PRAXIS, x.size());

	//lower and upper bounds of vertex coordinates
	vector<double> lb = computeLowerImageBoundConstraints(x);
	vector<double> ub = computeUpperImageBoundConstraints(x);
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	//minimize objective function
	opt.set_min_objective(Solver::wrapperOptFunc, this);

	// convergence criteria
	//opt.set_xtol_abs(0.5);
	opt.set_maxtime(120);

	double minf;
	nlopt::result result = opt.optimize(x, minf);
	
	cout << ">> Solution found after " << iterationCount << " iterations" << endl;

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
		tmp.vertices.at(i).x = (int) tmp.vertices.at(i).x * scaleX;
		tmp.vertices.at(i).y = (int) tmp.vertices.at(i).y * scaleY;
	}

	for (unsigned int i = 0; i < tmp.quads.size(); i++)
	{
		tmp.quads.at(i).v1.x = (int) tmp.quads.at(i).v1.x * scaleX;
		tmp.quads.at(i).v1.y = (int) tmp.quads.at(i).v1.y * scaleY;

		tmp.quads.at(i).v2.x = (int) tmp.quads.at(i).v2.x * scaleX;
		tmp.quads.at(i).v2.y = (int) tmp.quads.at(i).v2.y * scaleY;

		tmp.quads.at(i).v3.x = (int) tmp.quads.at(i).v3.x * scaleX;
		tmp.quads.at(i).v3.y = (int) tmp.quads.at(i).v3.y * scaleY;

		tmp.quads.at(i).v4.x = (int) tmp.quads.at(i).v4.x * scaleX;
		tmp.quads.at(i).v4.y = (int) tmp.quads.at(i).v4.y * scaleY;
	}

	for (unsigned int i = 0; i < tmp.edges.size(); i++)
	{
		tmp.edges.at(i).src.x = (int) tmp.edges.at(i).src.x * scaleX;
		tmp.edges.at(i).src.y = (int) tmp.edges.at(i).src.y * scaleY;

		tmp.edges.at(i).dest.x = (int) tmp.edges.at(i).dest.x * scaleX;
		tmp.edges.at(i).dest.y = (int) tmp.edges.at(i).dest.y * scaleY;
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
			sum1 += vTv(oldQuad.v2 - oldQuad.v3, newQuad.v2 - newQuad.v3);
			sum2 += sqr(Helper::euclideanNorm(oldQuad.v2 - oldQuad.v3));
		}
		else if (i == 2)
		{
			sum1 += vTv(oldQuad.v3 - oldQuad.v4, newQuad.v3 - newQuad.v4);
			sum2 += sqr(Helper::euclideanNorm(oldQuad.v3 - oldQuad.v4));
		}
		else
		{
			sum1 += vTv(oldQuad.v4 - oldQuad.v1, newQuad.v4 - newQuad.v1);
			sum2 += sqr(Helper::euclideanNorm(oldQuad.v4 - oldQuad.v1));
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
			v.x = v.x * sf;
			v.y = v.y * sf;

			du += sqr(Helper::euclideanNorm(_v - v));
		}
		else if (i == 1)
		{
			Vertex _v = newQuad.v2 - newQuad.v3;
			Vertex v = oldQuad.v2 - oldQuad.v3;
			v.x = v.x * sf;
			v.y = v.y * sf;

			du += sqr(Helper::euclideanNorm(_v - v));
		}
		else if (i == 2)
		{
			Vertex _v = newQuad.v3 - newQuad.v4;
			Vertex v = oldQuad.v3 - oldQuad.v4;
			v.x = v.x * sf;
			v.y = v.y * sf;

			du += sqr(Helper::euclideanNorm(_v - v));
		}
		else
		{
			Vertex _v = newQuad.v4 - newQuad.v1;
			Vertex v = oldQuad.v4 - oldQuad.v1;
			v.x = v.x * sf;
			v.y = v.y * sf;

			du += sqr(Helper::euclideanNorm(_v - v));
		}
	}

	return du;
}

double Solver::totalQuadEnergy(Mesh &newMesh)
{
	double du = 0.0;

	// assuming #quads in oldmesh = #quads in new mesh
	for (unsigned int i = 0; i < originalMesh.quads.size(); i++)
	{
		// calculate quad scale factor with the initial guess
		double sf = calculateQuadScale(originalMesh.quads.at(i), tmp.quads.at(i));
		double duf = quadEnergy(originalMesh.quads.at(i), newMesh.quads.at(i), sf);

		// du = du + wf * duf
		du += saliencyWeightMapping.at(i).first * duf;
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

		double lij = calculateLengthRatio(originalMesh.edges.at(i), tmp.edges.at(i));
		v.x = v.x * lij;
		v.y = v.y * lij;

		dl += sqr(Helper::euclideanNorm(_v - v));
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

	cout << "Iteration: " << iterationCount << endl;

	doubleVecToMesh(x, deformedMesh);

	double edgeEnergy = totalEdgeEnergy(deformedMesh);
	double quadEnergy = totalQuadEnergy(deformedMesh);

	double res = edgeEnergy + quadEnergy;

	return res;
}

double Solver::vTv(Vertex v1, Vertex v2)
{
	return (double) (v1.x * v2.x + v1.y * v2.y);
}

vector<double> Solver::meshToDoubleVec(Mesh &m)
{
	vector<double> x;

	for (unsigned int i = 0; i < m.vertices.size(); i++)
	{
		x.push_back(m.vertices.at(i).x);
		x.push_back(m.vertices.at(i).y);
	}

	return x;
}

void Solver::doubleVecToMesh(const vector<double> &x, Mesh &result)
{
	// clear mesh
	result.vertices.clear();
	result.edges.clear();
	result.quads.clear();

	// vertices
	for (unsigned int i = 0; i < x.size(); i += 2)
	{
		Vertex v;
		v.x = x.at(i);
		v.y = x.at(i + 1);

		result.vertices.push_back(v);
	}

	int xfac, yfac;
	int xSize = result.vertices.at(1).x;
	int ySize = result.vertices.at(2).y;

	// quads and edges
	for (unsigned int i = 0; i < QUAD_NUMBER_TOTAL; i++)
	{
		Quad q;
		Edge e1, e2, e3, e4;

		xfac = (int) i / QUAD_NUMBER_X;
		yfac = i % QUAD_NUMBER_Y;

		q.v1.x = xfac * xSize;
		q.v1.y = yfac * ySize;

		q.v2.x = (xfac + 1) * xSize;
		q.v2.y = yfac * ySize;

		q.v3.x = xfac * xSize;
		q.v3.y = (yfac + 1) * ySize;

		q.v4.x = (xfac + 1) * xSize;
		q.v4.y = (yfac + 1) * ySize;

		e1.src = q.v1;
		e1.dest = q.v2;
		e2.src = q.v2;
		e2.dest = q.v4;
		e3.src = q.v4;
		e3.dest = q.v3;
		e4.src = q.v3;
		e4.dest = q.v1;

		result.quads.push_back(q);

		if (i == 0)
		{
			result.edges.push_back(e1);
			result.edges.push_back(e2);
			result.edges.push_back(e3);
			result.edges.push_back(e4);
		}
		else
		{
			if (xfac == 0)
			{
				// don't add front edge of a quad since it's already part of the mesh
				result.edges.push_back(e2);
				result.edges.push_back(e3);
				result.edges.push_back(e4);
			}
			else
			{
				if (yfac == 0)
				{
					// don't add the left-hand edge of a quad since it's redundant
					result.edges.push_back(e1);
					result.edges.push_back(e2);
					result.edges.push_back(e3);
				}
				else
				{
					// don't add top and left-hand edge since they are redundant
					result.edges.push_back(e2);
					result.edges.push_back(e3);
				}
			}
		}
	}
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
				if (x.at(i) <= 1e-3)
					lb.at(i) = 0.0;
				else if (x.at(i) >= oldSize.width - originalQuadWidth && x.at(i) <= oldSize.width)
					lb.at(i) = (double) newSize.width;
				else
					lb.at(i) = 0.0;
			}
			else
			{
				// y-coordinates
				if(x.at(i) <= 1e-3)
					lb.at(i) = 0.0;
				else if (x.at(i) >= oldSize.height - originalQuadHeight && x.at(i) <= oldSize.height)
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