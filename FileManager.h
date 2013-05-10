#include "stdafx.h"

struct FileManager
{
	static BOOL mkDir(const char* &path);
	static void saveGrid(const string fileName, const string dir, const Mesh &m, const Size &s);
	static void saveMat(const string fileName, const string dir, const Mat &mat);
};