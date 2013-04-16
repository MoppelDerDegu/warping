#pragma once

#include "stdafx.h"

class Solver
{
public:
	Solver(void);
	~Solver(void);
	Mesh solveImageProblem(Mesh &m, Size &newSize, Size &originalSize, vector<pair<float, Quad>> &wfMap);
private:
	Mesh originalMesh;
	Mesh deformedMesh;
	Mesh tmp;
	vector<pair<float, Quad>> saliencyWeightMapping; //maps saliency weights to quads
	void initialGuess(Size &newSize, Size &originalSize);
	double calculateLengthRatio(Edge &oldEdge, Edge &newEdge); // unknown in equation (4)
	double calculateQuadScale(Quad &oldQuad, Quad &newQuad); // Wang et al. 2008 equation (2)
	double quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf); // Wang et al. 2008 equation (1)
	double totalQuadEnergy(Mesh &newMesh); // Wang et al. 2008 equation (3) 
	double totalEdgeEnergy(Mesh &newMesh); // Wang et al. 2008 equation (4)
	double imageObjFunc(const vector<double> &x, vector<double> &grad, void *myFuncData); // Wang et al. 2008 equation (5)
	double imageConstraint(const vector<double> &x, vector<double> &grad, void *data); // constraints for the optimization problem
	vector<double> meshToDoubleVec(Mesh &m);
	void doubleVecToMesh(vector<double> &x, Mesh &result);
	double vTv(Vertex v1, Vertex v2); // returns v1^tr * v2
};

