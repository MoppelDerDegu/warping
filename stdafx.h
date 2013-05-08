// stdafx.h : Includedatei für Standardsystem-Includedateien
// oder häufig verwendete projektspezifische Includedateien,
// die nur in unregelmäßigen Abständen geändert werden.
//

#ifndef HEADER_H
#define HEADER_H

#pragma once
#pragma warning (disable : 4996)
#pragma warning (disable : 4244)
#pragma warning (disable : 4091)

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <Windows.h>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>


using namespace cv;
using namespace std;

#define GRADIENT_SOBEL 1
#define GRADIENT_SCHARR 2
#define GRADIENT_SIMPLE 3

// number of quads in the image in x and y direction respectively
#define QUAD_NUMBER_X 5
#define QUAD_NUMBER_Y 5
#define QUAD_NUMBER_TOTAL QUAD_NUMBER_X * QUAD_NUMBER_Y

typedef struct Vertex
{
	int x, y;
	Vertex operator+ (Vertex);
	Vertex operator- (Vertex);
	bool operator== (Vertex &v);
};

typedef struct Edge
{
	Vertex src;
	Vertex dest;
	bool operator== (Edge &e);
};

// vertex and edge numeration of a quad is like this
/*        e1
	v1 --------- v2
	|			 |
 e4	|			 | e2
	|			 |
	v3 --------- v4
		  e3
*/
typedef struct Quad
{
	Vertex v1;
	Vertex v2;
	Vertex v3;
	Vertex v4;
};

typedef struct Mesh
{
	vector<Vertex> vertices;
	vector<Edge> edges;
	vector<Quad> quads;
};

template<typename T> inline T sqr(T x) { return x * x;}
template<class T> inline T vecDist3(const Vec<T, 3> &v1, const Vec<T, 3> &v2) {return sqrt(sqr(v1[0] - v2[0])+sqr(v1[1] - v2[1])+sqr(v1[2] - v2[2]));}
template<class T> inline T vecSqrDist3(const Vec<T, 3> &v1, const Vec<T, 3> &v2) {return sqr(v1[0] - v2[0])+sqr(v1[1] - v2[1])+sqr(v1[2] - v2[2]);}
template<class T1, class T2> inline void operator /= (Vec<T1, 3> &v1, const T2 v2) { v1[0] /= v2; v1[1] /= v2; v1[2] /= v2; }
template<class T> inline T pntSqrDist(const Point_<T> &p1, const Point_<T> &p2) {return sqr(p1.x - p2.x) + sqr(p1.y - p2.y);}


// TODO: Hier auf zusätzliche Header, die das Programm erfordert, verweisen.
#endif