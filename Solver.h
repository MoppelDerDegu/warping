#pragma once

#include "stdafx.h"

class Solver
{
protected:
	unsigned int iterationCount;

	double calculateLengthRatio(Edge &oldEdge, Edge &newEdge); // unknown in equation (4)
	double calculateQuadScale(Quad &oldQuad, Quad &newQuad); // Wang et al. 2008 equation (2)
	double quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf); // Wang et al. 2008 equation (1)
	double totalQuadEnergy(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Quad, float>> &scalingFactors, const vector<pair<float, Quad>> &saliencyWeightMapping); // Wang et al. 2008 equation (3) 
	double totalEdgeEnergy(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Edge, float>> &edgeLengthRatios); // Wang et al. 2008 equation (4)
	double totalRedistributionEnergy(Mesh &newMesh, vector<pair<Edge, float>> edgeSaliency); // Wang et al. 2008 equation (9)

	// initial guess of the new vertex coordinates, i.e. linear scaling according to the new image size
	void initialGuess(Mesh &src, Mesh& dest, Size &newSize, Size &originalSize);

	vector<double> computeLowerImageBoundConstraints(const vector<double> &x, const Size size);
	vector<double> computeUpperImageBoundConstraints(const vector<double> &x, const Size size);

	void calculateEdgeLengthRatios(Mesh &originalMesh, Mesh &deformedMesh, vector<pair<Edge, float>> &edgeLengthRatios);
	void calculateOptimalScaleFactors(Mesh &originalMesh, Mesh &deformedMesh, vector<pair<Quad, float>> &scalingFactors);
};