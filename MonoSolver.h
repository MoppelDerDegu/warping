#pragma once

#include "stdafx.h"
#include "Solver.h"

class MonoSolver : public Solver
{
public:
	MonoSolver(Size &originalSize);
	~MonoSolver(void);
	Mesh redistributeQuads(Mesh &originalMesh, vector<pair<float, Quad>> &wfMap); //draws quads towards salient regions
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
	double quadscale;

	double imageObjFunc(const vector<double> &x, vector<double> &grad);
	double redistributeObjFunc(const vector<double> &x, vector<double> &grad); 
};

