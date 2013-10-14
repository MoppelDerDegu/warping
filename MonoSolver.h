#pragma once

#include "stdafx.h"
#include "Solver.h"

// Solves the mesh optimization problem for a 2D image
class MonoSolver : public Solver
{
public:
	MonoSolver(Size &originalSize);
	~MonoSolver(void);
	Mesh solveImageProblem(Mesh &contentAwareMesh, Mesh &originalMesh, Size &newSize, vector<pair<float, Quad>> &wfMap);
	
	Mesh getDeformedMesh(); // returns the result of the mesh optimization
	Mesh getInitialGuess(); // returns the initial guess of the mesh optimization

private:
	Mesh originalMesh; // uniform mesh over the original image
	Mesh contentAwareMesh; // content aware mesh over the original image
	Mesh deformedMesh; // result mesh after optimization
	Mesh tmp; // stores the initial guess
	vector<pair<float, Quad>> saliencyWeightMapping; // maps saliency weights to quads
	Size oldSize;
	Size newSize;
	vector<pair<Edge, float>> edgeSaliency; // maps average saliency weights to edges
	vector<pair<Edge, float>> edgeLengthRatios; // lij
	vector<pair<Quad, float>> scalingFactors; // sf
	double quadscale;

	static double wrapperImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data); // wrapper method for the objective function that can be passed to the NLopt library
	double imageObjFunc(const vector<double> &x, vector<double> &grad); // objective function for the mesh optimization
};

