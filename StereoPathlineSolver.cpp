#include "StereoPathlineSolver.h"
#include "PathlineManager.h"


StereoPathlineSolver::StereoPathlineSolver(unsigned int maxEval, int frame, PathlineSets &leftOptimizedPathlines, PathlineSets &rightOptimizedPathlines, PathlineSets &leftOriginalPathlines, PathlineSets &rightOriginalPathlines)
{
	iterationCount = 0;
	this->maxEval = maxEval;
	this->frame = frame;
	this->leftOptimizedPathlines = leftOptimizedPathlines;
	this->rightOptimizedPathlines = rightOptimizedPathlines;
	this->leftOriginalPathlines = leftOriginalPathlines;
	this->rightOriginalPathlines = rightOriginalPathlines;
}

StereoPathlineSolver::~StereoPathlineSolver(void)
{
}

pair<Mesh, Mesh> StereoPathlineSolver::solveStereoImageProblem(Mesh &originalLeft, Mesh &originalRight, Size &oldSize, Size &newSize, vector<pair<float, Quad>> &wfMapLeft, vector<pair<float, Quad>> &wfMapRight)
{
	cout << "> Solving stereo image optimization problem regarding pathlines..." << endl;

	MeshManager* mm = MeshManager::getInstance();
	PathlineManager* pm = PathlineManager::getInstance();

	// initialization
	this->saliencyWeightMappingLeft = wfMapLeft;
	this->saliencyWeightMappingRight = wfMapRight;
	this->originalLeft = originalLeft;
	this->originalRight = originalRight;

	// map pathlines to quads
	pm->mapPathlinesToQuads(frame, leftOriginalPathlines, originalLeft, leftQuadMapping);
	pm->mapPathlinesToQuads(frame, rightOriginalPathlines, originalRight, rightQuadMapping);

	// get set of pathlines passing this' frame
	pm->getLinesContainingFrame(leftOriginalPathlines, frame, leftOriginalLines);
	pm->getLinesContainingFrame(rightOriginalPathlines, frame, rightOriginalLines);
	pm->getLinesContainingFrame(leftOptimizedPathlines, frame, leftOptimizedLines);
	pm->getLinesContainingFrame(rightOptimizedPathlines, frame, rightOptimizedLines);

	// determine all pathline points in this' frame
	pm->getPointsInFrame(leftOriginalLines, frame, leftOriginalPathlinePoints);
	pm->getPointsInFrame(rightOriginalLines, frame, rightOriginalPathlinePoints);
	pm->getPointsInFrame(leftOptimizedLines, frame, leftOptimizedPathlinePoints);
	pm->getPointsInFrame(rightOptimizedLines, frame, rightOptimizedPathlinePoints);

	// left size = right size
	Size oldLeftSize = Size(oldSize.width / 2, oldSize.height);
	Size newLeftSize = Size(newSize.width / 2, newSize.height);

	// initial guess for left and right view
	initialGuess(originalLeft, initialLeft, newLeftSize, oldLeftSize);
	initialGuess(originalRight, initialRight, newLeftSize, oldLeftSize);

	// copy initial left and right to result
	deformedLeft = mm->deepCopyMesh(initialLeft);
	deformedRight = mm->deepCopyMesh(initialRight);

	// TODO formulate optimization problem and solve it
}