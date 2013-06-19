#pragma once

#include "solver.h"

class StereoSolver : public Solver
{
public:
	StereoSolver(void);
	~StereoSolver(void);

	pair<Mesh, Mesh> solveStereoImageProblem(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight);
	pair<Mesh, Mesh> solveStereoImageProblemSeperately(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight);
	Mesh getInitialLeft();
	Mesh getInitialRight();

private:
	Mesh initialLeft;
	Mesh initialRight;
	Mesh deformedLeft;
	Mesh deformedRight;
	Mesh originalLeft;
	Mesh originalRight;
	
	Mesh deformedLeftXOnly;
	Mesh deformedLeftYOnly;
	Mesh deformedRightXOnly;
	Mesh deformedRightYOnly;

	int splitIndex;
	int splitIndexSeperate;

	vector<pair<float, Quad>> saliencyWeightMappingLeft;
	vector<pair<float, Quad>> saliencyWeightMappingRight;
	vector<pair<Edge, float>> edgeLengthRatiosLeft; // lij left view
	vector<pair<Edge, float>> edgeLengthRatiosRight; // lij right view
	vector<pair<Quad, float>> scalingFactorsLeft; // sf left view
	vector<pair<Quad, float>> scalingFactorsRight; // sf right view

	static double wrapperStereoImageObjectiveFunc(const vector<double> &x, vector<double> &grad, void *my_func_data);
	static double wrapperStereoImageObjectiveX(const vector<double> &x, vector<double> &grad, void *my_func_data);
	static double wrapperStereoImageObjectiveY(const vector<double> &x, vector<double> &grad, void *my_func_data);

	double stereoImageObjFunc(const vector<double> &x, vector<double> &grad);
	double stereoImageObjFuncX(const vector<double> &x, vector<double> &grad);
	double stereoImageObjFuncY(const vector<double> &x, vector<double> &grad);

	double stereoEnergy(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight); // Yoo et al. 2013 equation (6)
	double stereoEnergyX(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight);
	double stereoEnergyY(Mesh &originalLeft, Mesh &originalRight, Mesh &newLeft, Mesh &newRight);

	double quadEnergyX(Quad &oldQuad, Quad &newQuad, const double sf);
	double quadEnergyY(Quad &oldQuad, Quad &newQuad, const double sf);
	double totalQuadEnergyX(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Quad, float>> &scalingFactors, const vector<pair<float, Quad>> &saliencyWeightMapping);
	double totalQuadEnergyY(Mesh &originalMesh, Mesh &newMesh, const vector<pair<Quad, float>> &scalingFactors, const vector<pair<float, Quad>> &saliencyWeightMapping);

	vector<double> computeLowerXBoundConstraints(const vector<double> &x, const Size size);
	vector<double> computeUpperXBoundConstraints(const vector<double> &x, const Size size);

	vector<double> computeLowerYBoundConstraints(const vector<double> &x, const Size size);
	vector<double> computeUpperYBoundConstraints(const vector<double> &x, const Size size);
};

