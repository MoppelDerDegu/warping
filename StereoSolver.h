#pragma once

#include "solver.h"

class StereoSolver : public Solver
{
public:
	StereoSolver()
	{
	}
	StereoSolver(unsigned int);
	~StereoSolver(void);

	// deforms the left and right mesh according to the saliency map
	pair<Mesh, Mesh> solveStereoImageProblem(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight);

	Mesh getInitialLeft(); // returns the initial guess for the left mesh
	Mesh getInitialRight(); // returns the initial guess for the right mesh

protected:
	Mesh initialLeft;
	Mesh initialRight;
	Mesh deformedLeft;
	Mesh deformedRight;
	Mesh originalLeft;
	Mesh originalRight;

	unsigned int maxEval; // maximum number of alternating optimization steps

	int splitIndex;

	vector<pair<float, Quad>> saliencyWeightMappingLeft;
	vector<pair<float, Quad>> saliencyWeightMappingRight;
	vector<pair<Edge, float>> edgeLengthRatiosLeft; // edge length ratios left view
	vector<pair<Edge, float>> edgeLengthRatiosRight; // edge length ratios right view
	vector<pair<Quad, float>> scalingFactorsLeft; // quad scaling factors left view
	vector<pair<Quad, float>> scalingFactorsRight; // quad scaling factors right view

	static double wrapperStereoImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data); // wrapper function that is passed to NLopt library

	double stereoImageObjFunc(const vector<double> &x, vector<double> &grad);

	double stereoEnergy(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight); // Yoo et al. 2013 equation (6)
};

