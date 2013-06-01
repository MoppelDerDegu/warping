#pragma once

#include "stdafx.h"

class Solver
{
public:
	Solver(Size &originalSize);
	~Solver(void);
	Mesh redistributeQuads(Mesh &m, vector<pair<float, Quad>> &wfMap); //draws quads towards salient regions
	Mesh solveImageProblem(Mesh &contentAwareMesh, Mesh &originalMesh, Size &newSize, vector<pair<float, Quad>> &wfMap);
	static double wrapperImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data);
	static double wrapperRedistributeObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data);
	Mesh getDeformedMesh();
	Mesh getInitialGuess();

private:
	Mesh originalMesh; // uniform mesh over the original image
	Mesh contentAwareMesh; // content aware mesh over the original image
	Mesh deformedMesh; // result mesh after optimization
	Mesh tmp; // stores the initial guess
	vector<pair<float, Quad>> saliencyWeightMapping; // maps saliency weights to quads
	Size oldSize;
	Size newSize;
	unsigned int iterationCount;
	vector<pair<Edge, float>> edgeSaliency; // maps average saliency weights to edges
	vector<pair<Edge, float>> edgeLengthRatios; // lij
	vector<pair<Quad, float>> scalingFactors; // sf

	void initialGuess(Size &newSize, Size &originalSize);
	double calculateLengthRatio(Edge &oldEdge, Edge &newEdge); // unknown in equation (4)
	double calculateQuadScale(Quad &oldQuad, Quad &newQuad); // Wang et al. 2008 equation (2)
	double quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf); // Wang et al. 2008 equation (1)
	double totalQuadEnergy(Mesh &newMesh); // Wang et al. 2008 equation (3) 
	double totalEdgeEnergy(Mesh &newMesh); // Wang et al. 2008 equation (4)
	double imageObjFunc(const vector<double> &x, vector<double> &grad); // Wang et al. 2008 equation (5)
	double redistributeObjFunc(const vector<double> &x, vector<double> &grad); 
	double totalRedistributionEnergy(Mesh &newMesh); // Wang et al. 2008 equation (9)
	vector<double> computeLowerImageBoundConstraints(const vector<double> &x, const Size size);
	vector<double> computeUpperImageBoundConstraints(const vector<double> &x, const Size size);
	void calculateEdgeLengthRatios();
	void calculateOptimalScaleFactors();
	void calculateQuadScale();
};

