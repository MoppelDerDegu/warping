#include "Solver.h"
#include "Helper.h"


Solver::Solver(void)
{
}


Solver::~Solver(void)
{
}

Mesh Solver::solveImageProblem(Mesh &m, Size &newSize, Size &originalSize)
{
	this->originalMesh = m;
	initialGuess(newSize, originalSize);

	// TODO solve optimization problem
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
		Vertex v = tmp.vertices.at(i);
		v.x = (int) (v.x * scaleX);
		v.y = (int) (v.y * scaleY);
	}
}

double Solver::calculateLengthRatio(Edge &oldEdge, Edge &newEdge)
{
	return (Helper::euclideanNorm(newEdge.src - newEdge.dest)) / (Helper::euclideanNorm(oldEdge.src - oldEdge.dest));
}

double Solver::calculateQuadScale(const Quad &q)
{
	double sf;

	// TODO

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

double Solver::totalQuadEnergy(const Mesh &m)
{
	// TODO
}

double Solver::totalEdgeEnergy(const Mesh &m)
{
	// TODO
}

double Solver::imageObjFunc(const vector<double> &x, vector<double> &grad, void *myFuncData)
{
	// TODO 
}