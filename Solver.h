#pragma once

#include "stdafx.h"

// base classes that solve mesh optimization problems
class Solver
{
protected:
	unsigned int iterationCount;

	double calculateLengthRatio(Edge &oldEdge, Edge &newEdge); // unknown in equation (4)
	double calculateQuadScale(Quad &oldQuad, Quad &newQuad); // equation 3.6
	double quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf); // Wang et al. 2008 equation (1)
	double totalQuadEnergy(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Quad, float>> &scalingFactors, const vector<pair<float, Quad>> &saliencyWeightMapping); // Wang et al. 2008 equation (3) 
	double totalEdgeEnergy(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Edge, float>> &edgeLengthRatios); // Wang et al. 2008 equation (4)

	// initial guess of the new vertex coordinates, i.e. linear scaling according to the new image size
	void initialGuess(Mesh &src, Mesh& dest, Size &newSize, Size &originalSize);

	vector<double> computeLowerImageBoundConstraints(const vector<double> &x, const Size size);
	vector<double> computeUpperImageBoundConstraints(const vector<double> &x, const Size size);

	void calculateEdgeLengthRatios(Mesh &originalMesh, Mesh &deformedMesh, vector<pair<Edge, float>> &edgeLengthRatios); // calculates edge length ratios before and after deformation
	void calculateOptimalScaleFactors(Mesh &originalMesh, Mesh &deformedMesh, vector<pair<Quad, float>> &scalingFactors); // calculates the optimal scaling factors for all quads
};