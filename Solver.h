#pragma once

#include "stdafx.h"

class Solver
{
public:
	Solver(void);
	~Solver(void);
	Mat solveImageProblem(Mesh &m, Size &newSize, Size &originalSize, vector<pair<float, Quad>> &wfMap, IplImage* img);
	static double wrapperOptFunc(const vector<double> &x, vector<double> &grad, void *my_func_data);
	Mesh getDeformedMesh();
private:
	Mat destImage;
	Mesh originalMesh;
	Mesh deformedMesh; // result mesh after optimization
	Mesh tmp; // stores the initial guess
	vector<pair<float, Quad>> saliencyWeightMapping; //maps saliency weights to quads
	Size oldSize;
	Size newSize;
	unsigned int iterationCount;
	void initialGuess(Size &newSize, Size &originalSize);
	double calculateLengthRatio(Edge &oldEdge, Edge &newEdge); // unknown in equation (4)
	double calculateQuadScale(Quad &oldQuad, Quad &newQuad); // Wang et al. 2008 equation (2)
	double quadEnergy(Quad &oldQuad, Quad &newQuad, const double sf); // Wang et al. 2008 equation (1)
	double totalQuadEnergy(Mesh &newMesh); // Wang et al. 2008 equation (3) 
	double totalEdgeEnergy(Mesh &newMesh); // Wang et al. 2008 equation (4)
	double imageObjFunc(const vector<double> &x, vector<double> &grad); // Wang et al. 2008 equation (5)
	vector<double> computeLowerImageBoundConstraints(const vector<double> &x);
	vector<double> computeUpperImageBoundConstraints(const vector<double> &x);
	double vTv(Vertex v1, Vertex v2); // returns v1^tr * v2
	vector<double> meshToDoubleVec(Mesh &m);
	void doubleVecToMesh(const vector<double> &x, Mesh &result);
	void initialScale();
	void warp(int interpolation = INTER_LINEAR); //warps the destImage according to deformedMesh
};

