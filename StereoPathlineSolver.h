#pragma once

#include "stereosolver.h"

class StereoPathlineSolver : public StereoSolver
{
public:
	StereoPathlineSolver(unsigned int maxEval, int frame, PathlineSets &leftOptimizedPathlines, PathlineSets &rightOptimizedPathlines, PathlineSets &leftOriginalPathlines, PathlineSets &rightOriginalPathlines);
	~StereoPathlineSolver(void);

	pair<Mesh, Mesh> solveStereoImageProblem(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight);

private:
	PathlineSets leftOptimizedPathlines, rightOptimizedPathlines, leftOriginalPathlines, rightOriginalPathlines;

	// set of pathline points in the frame
	vector<Point2f> leftOriginalPathlinePoints, rightOriginalPathlinePoints, leftOptimizedPathlinePoints, rightOptimizedPathlinePoints;

	// set of pathlines that pass the frame
	vector<Pathline> leftOriginalLines, rightOriginalLines, leftOptimizedLines, rightOptimizedLines;

	// maps a pathline to the quad it belongs to, the key is the index of the adress where the pathline is stored in the respective collection. Same goes for the value.
	map<int, int> leftQuadMapping, rightQuadMapping;

	int frame;
};

