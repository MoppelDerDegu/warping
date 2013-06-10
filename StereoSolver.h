#pragma once

#include "solver.h"

class StereoSolver : public Solver
{
public:
	StereoSolver(void);
	~StereoSolver(void);

	pair<Mesh, Mesh> solveStereoImageProblem(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight);
	Mesh getInitialLeft();
	Mesh getInitialRight();

private:
	Mesh initialLeft;
	Mesh initialRight;
	Mesh deformedLeft;
	Mesh deformedRight;
	Mesh originalLeft;
	Mesh originalRight;

	int splitIndex;
	vector<pair<float, Quad>> saliencyWeightMappingLeft;
	vector<pair<float, Quad>> saliencyWeightMappingRight;
	vector<pair<Edge, float>> edgeLengthRatiosLeft; // lij left view
	vector<pair<Edge, float>> edgeLengthRatiosRight; // lij right view
	vector<pair<Quad, float>> scalingFactorsLeft; // sf left view
	vector<pair<Quad, float>> scalingFactorsRight; // sf right view

	static double wrapperStereoImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data);
	double stereoImageObjFunc(const vector<double> &x, vector<double> &grad);
	double stereoEnergy(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight); // Yoo et al. 2013 equation (6)
};

