#pragma once

#include "stdafx.h"

class Solver
{
public:
	Solver(void);
	~Solver(void);
	Mesh solveImageProblem(Mesh &m, Size &newSize, Size &originalSize);
private:
	Mesh originalMesh;
	Mesh deformedMesh;
	Mesh tmp;
	void initialGuess(Size &newSize, Size &originalSize);
	double calculateLengthRatio(Edge &oldEdge, Edge &newEdge); // unknown in equation (4)
	double calculateQuadScale(const Quad &q); // Wang et al. 2008 equation (2)
	double quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf); // Wang et al. 2008 equation (1)
	double totalQuadEnergy(const Mesh &m); // Wang et al. 2008 equation (3) 
	double totalEdgeEnergy(const Mesh &m); // Wang et al. 2008 equation (4)
	double imageObjFunc(const vector<double> &x, vector<double> &grad, void *myFuncData); // Wang et al. 2008 equation (5)
};

