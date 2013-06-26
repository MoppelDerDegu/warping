#include "stdafx.h"

struct FileManager
{
	static BOOL mkDir(const char* &path);
	static void saveMeshAsImage(const string fileName, const string dir, const Mesh &m, const Size &s);
	static void saveMat(const string fileName, const string dir, const Mat &mat);
	static void saveMeshAsText(const string fileName, const string dir, Mesh &mesh);
	static Mesh loadMesh(const string fileName);
	static void saveMeshROIAsImage(const string fileName, const string dir, const Mesh &m, const Size &s);
	static void saveMeshesAsText(const string fileName, const string dir, vector<Mesh> &meshes);
	static vector<Mesh> loadMeshes(const string fileName);
};