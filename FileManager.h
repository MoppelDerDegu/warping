#include "stdafx.h"

struct FileManager
{
	static BOOL mkDir(const char* &path); // creates a directory on the file system
	static void saveMeshAsImage(const string &fileName, const string &dir, const Mesh &m, const Size &s); // saves a mesh as an image
	static void saveMat(const string &fileName, const string &dir, const Mat &mat); // saves a mat object
	static void saveMeshAsText(const string &fileName, const string &dir, Mesh &mesh); // saves a mesh in a txt file
	static Mesh loadMesh(const string &fileName); // loads a mesh from a txt file
	static void saveMeshROIAsImage(const string &fileName, const string &dir, const Mesh &m, const Size &s); // saves a mesh and the ROIs placed over each quad as an image
	static void saveMeshesAsText(const string &fileName, const string &dir, vector<Mesh> &meshes); // saves a collection of meshes in a txt file
	static vector<Mesh> loadMeshes(const string &fileName); // load a collection of meshes from a txt file
	static void savePathlines(const string &fileName, const string &dir, const vector<Pathline> &pathlines); // saves a vector of pathlines in a txt file
	static vector<Pathline> loadPathlines(const string &fileName); // loads a vector of pathlines from a txt file
};