#include "Solver.h"
#include "WarpingMath.h"
#include "MeshManager.h"

void Solver::calculateEdgeLengthRatios(Mesh &originalMesh, Mesh &deformedMesh, vector<pair<Edge, float>> &edgeLengthRatios)
{
	edgeLengthRatios.clear();

	for (unsigned int i = 0; i < deformedMesh.edges.size(); i++)
	{
		pair<Edge, float> p;
		p.first = deformedMesh.edges.at(i);
		p.second = calculateLengthRatio(originalMesh.edges.at(i), deformedMesh.edges.at(i));

		edgeLengthRatios.push_back(p);
	}
}

void Solver::calculateOptimalScaleFactors(Mesh &originalMesh, Mesh &deformedMesh, vector<pair<Quad, float>> &scalingFactors)
{
	scalingFactors.clear();

	for (unsigned int i = 0; i < deformedMesh.quads.size(); i++)
	{
		pair<Quad, float> p;
		p.first = deformedMesh.quads.at(i);
		p.second = calculateQuadScale(originalMesh.quads.at(i), deformedMesh.quads.at(i));

		scalingFactors.push_back(p);
	}
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
	
	return du;
}

double Solver::totalQuadEnergy(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Quad, float>> &scalingFactors, const vector<pair<float, Quad>> &saliencyWeightMapping)
{
	double du = 0.0;

	// assuming #quads in oldmesh == #quads in new mesh
	for (unsigned int i = 0; i < originalMesh.quads.size(); i++)
	{
		double sf = scalingFactors.at(i).second;
		double duf = quadEnergy(originalMesh.quads.at(i), newMesh.quads.at(i), sf);

		// du = du + wf * duf
		du += ((saliencyWeightMapping.at(i).first) * duf);
	}

	return du;
}

double Solver::totalEdgeEnergy(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Edge, float>> &edgeLengthRatios)
{
	double dl = 0.0;

	for (unsigned int i = 0; i < originalMesh.edges.size(); i++)
	{	
		Vertex _v = newMesh.edges.at(i).src - newMesh.edges.at(i).dest;
		Vertex v = originalMesh.edges.at(i).src - originalMesh.edges.at(i).dest;
		
		double lij = edgeLengthRatios.at(i).second;
		v.x = WarpingMath::round(v.x * lij);
		v.y = WarpingMath::round(v.y * lij);
		
		dl += sqr(WarpingMath::euclideanNorm(_v - v));
	}

	return dl;
}

void Solver::initialGuess(Mesh &src, Mesh& dest, Size &newSize, Size &originalSize)
{
	MeshManager* mm = MeshManager::getInstance();

	int newWidth = newSize.width;
	int newHeight = newSize.height;
	int oldHeight = originalSize.height;
	int oldWidth = originalSize.width;

	float scaleX = (float) newWidth / (float) oldWidth;
	float scaleY = (float) newHeight / (float) oldHeight;

	dest = mm->deepCopyMesh(src);

	// scale vertices
	for (unsigned int i = 0; i < dest.vertices.size(); i++)
	{
		dest.vertices.at(i).x = WarpingMath::round(dest.vertices.at(i).x * scaleX);
		dest.vertices.at(i).y = WarpingMath::round(dest.vertices.at(i).y * scaleY);
	}

	// construct quads
	for (unsigned int i = 0; i < dest.quads.size(); i++)
	{
		dest.quads.at(i).v1.x = WarpingMath::round(dest.quads.at(i).v1.x * scaleX);
		dest.quads.at(i).v1.y = WarpingMath::round(dest.quads.at(i).v1.y * scaleY);

		dest.quads.at(i).v2.x = WarpingMath::round(dest.quads.at(i).v2.x * scaleX);
		dest.quads.at(i).v2.y = WarpingMath::round(dest.quads.at(i).v2.y * scaleY);

		dest.quads.at(i).v3.x = WarpingMath::round(dest.quads.at(i).v3.x * scaleX);
		dest.quads.at(i).v3.y = WarpingMath::round(dest.quads.at(i).v3.y * scaleY);

		dest.quads.at(i).v4.x = WarpingMath::round(dest.quads.at(i).v4.x * scaleX);
		dest.quads.at(i).v4.y = WarpingMath::round(dest.quads.at(i).v4.y * scaleY);
	}

	// construct edges
	for (unsigned int i = 0; i < dest.edges.size(); i++)
	{
		dest.edges.at(i).src.x = WarpingMath::round(dest.edges.at(i).src.x * scaleX);
		dest.edges.at(i).src.y = WarpingMath::round(dest.edges.at(i).src.y * scaleY);

		dest.edges.at(i).dest.x = WarpingMath::round(dest.edges.at(i).dest.x * scaleX);
		dest.edges.at(i).dest.y = WarpingMath::round(dest.edges.at(i).dest.y * scaleY);
	}
}