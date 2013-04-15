#include "Solver.h"
#include "Helper.h"


Solver::Solver(void)
{
}


Solver::~Solver(void)
{
}

Mesh Solver::solveImageProblem(Mesh &m, Size &newSize, Size &originalSize, vector<pair<float, Quad>> &wfMap)
{
	cout << ">> Solving Image Optimization Problem" << endl;

	this->originalMesh = m;
	this->saliencyWeightMapping = wfMap;
	initialGuess(newSize, originalSize);

	// TODO solve optimization problem

	return deformedMesh;
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

	for (unsigned int i = 0; originalMesh.edges.size(); i++)
	{
		Vertex _v = newMesh.edges.at(i).src - newMesh.edges.at(i).dest;
		Vertex v = originalMesh.edges.at(i).src - originalMesh.edges.at(i).dest;

		double lij = calculateLengthRatio(originalMesh.edges.at(i), newMesh.edges.at(i));
		v.x = v.x * lij;
		v.y = v.y * lij;

		dl += sqr(Helper::euclideanNorm(_v - v));
	}

	return dl;
}

double Solver::imageObjFunc(const vector<double> &x, vector<double> &grad, void *myFuncData)
{
	// TODO 

	return 0.0;
}

double Solver::vTv(Vertex v1, Vertex v2)
{
	return (double) (v1.x * v2.x + v1.y * v2.y);
}