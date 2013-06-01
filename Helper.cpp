#include "Helper.h"
#include "stdafx.h"
#include "WarpingMath.h"
#include "FileManager.h"

IplImage* Helper::getNthFrame(CvCapture* capture, int n)
{
	for(int i = 0; i <= n; i++)
	{
		if(cvQueryFrame(capture) == NULL)
			return NULL;
	}

	return cvQueryFrame(capture);
}

IplImage Helper::MatToIplImage(Mat& m)
{
	IplImage img = m;
	return img;
}

Mat Helper::IplImageToMat(IplImage* im)
{
	Mat m = im;
	return m;
}

string Helper::getImageType(int number)
{
    // find type
    int imgTypeInt = number % 8;
    string imgTypeString;

    switch (imgTypeInt)
    {
        case 0:
            imgTypeString = "8U";
            break;
        case 1:
            imgTypeString = "8S";
            break;
        case 2:
            imgTypeString = "16U";
            break;
        case 3:
            imgTypeString = "16S";
            break;
        case 4:
            imgTypeString = "32S";
            break;
        case 5:
            imgTypeString = "32F";
            break;
        case 6:
            imgTypeString = "64F";
            break;
        default:
            break;
    }

    // find channel
    int channel = (number / 8) + 1;

    stringstream type;
    type << "CV_" << imgTypeString << "C" << channel;

    return type.str();
}

Mesh Helper::deepCopyMesh(const Mesh &m)
{
	Mesh result;
	
	for (unsigned int i = 0; i < m.quads.size(); i++)
	
	{
		Quad f;
		Vertex v1;
		Vertex v2;
		Vertex v3;
		Vertex v4;
		
		v1.x = m.quads.at(i).v1.x;
		v1.y = m.quads.at(i).v1.y;
		v2.x = m.quads.at(i).v2.x;
		v2.y = m.quads.at(i).v2.y;
		v3.x = m.quads.at(i).v3.x;
		v3.y = m.quads.at(i).v3.y;
		v4.x = m.quads.at(i).v4.x;
		v4.y = m.quads.at(i).v4.y;

		f.v1 = v1;
		f.v2 = v2;
		f.v3 = v3;
		f.v4 = v4;

		result.quads.push_back(f);
	}

	for (unsigned int i = 0; i < m.edges.size(); i++)
	{
		Edge edge;
		edge.src = m.edges.at(i).src;
		edge.dest = m.edges.at(i).dest;
		
		result.edges.push_back(edge);
	}

	for (unsigned int i = 0; i < m.vertices.size(); i++)
	{
		result.vertices.push_back(m.vertices.at(i));
	}

	return result;
}

vector<double> Helper::meshToDoubleVec(Mesh &m)
{
	vector<double> x;

	for (unsigned int i = 0; i < m.vertices.size(); i++)
	{
		x.push_back(m.vertices.at(i).x);
		x.push_back(m.vertices.at(i).y);
	}

	return x;
}

void Helper::doubleVecToMesh(const vector<double> &x, Mesh &result)
{
	// clear mesh
	result.vertices.clear();
	result.edges.clear();
	result.quads.clear();

	// vertices
	for (unsigned int i = 0; i < x.size(); i += 2)
	{
		Vertex v;
		v.x = WarpingMath::round(x.at(i));
		v.y = WarpingMath::round(x.at(i + 1));

		result.vertices.push_back(v);
	}

	int vertexCount = 0;
	int diff = QUAD_NUMBER_Y * 2 + 1;
	int xfac, yfac;

	// quads and edges
	for (unsigned int i = 0; i < QUAD_NUMBER_TOTAL; i++)
	{
		Quad q;
		Edge e1, e2, e3, e4;

		xfac = (int) i / QUAD_NUMBER_X;
		yfac = i % QUAD_NUMBER_Y;

		if (vertexCount <= QUAD_NUMBER_Y * 2)
		{
			// first column of mesh
			q.v1 = result.vertices.at(vertexCount);
			q.v2 = result.vertices.at(vertexCount + 1);
			q.v3 = result.vertices.at(vertexCount + 2);
			q.v4 = result.vertices.at(vertexCount + 3);

			if ( vertexCount < (QUAD_NUMBER_Y - 1) * 2)
				vertexCount += 2;
			else
				vertexCount += 4;
		}
		else
		{
			if (xfac < 2)
			{
				// second column
				q.v1 = result.vertices.at(vertexCount - diff);
				q.v2 = result.vertices.at(vertexCount);
				q.v3 = result.vertices.at(vertexCount + 1 - (diff - 1));
				q.v4 = result.vertices.at(vertexCount + 1);
			}
			else
			{
				// all other columns
				q.v1 = result.vertices.at(vertexCount - diff);
				q.v2 = result.vertices.at(vertexCount);
				q.v3 = result.vertices.at(vertexCount + 1 - diff);
				q.v4 = result.vertices.at(vertexCount + 1);
			}

			if (i < QUAD_NUMBER_Y * 2)
				diff--;	

			if (yfac == QUAD_NUMBER_Y - 1)
				vertexCount += 2;
			else
				vertexCount++;
		}

		e1.src = q.v1;
		e1.dest = q.v2;
		e2.src = q.v2;
		e2.dest = q.v4;
		e3.src = q.v4;
		e3.dest = q.v3;
		e4.src = q.v3;
		e4.dest = q.v1;

		result.quads.push_back(q);

		if (i == 0)
		{
			result.edges.push_back(e1);
			result.edges.push_back(e2);
			result.edges.push_back(e3);
			result.edges.push_back(e4);
		}
		else
		{
			if (xfac == 0)
			{
				// don't add front edge of a quad since it's already part of the mesh
				result.edges.push_back(e2);
				result.edges.push_back(e3);
				result.edges.push_back(e4);
			}
			else
			{
				if (yfac == 0)
				{
					// don't add the left-hand edge of a quad since it's redundant
					result.edges.push_back(e1);
					result.edges.push_back(e2);
					result.edges.push_back(e3);
				}
				else
				{
					// don't add top and left-hand edge since they are redundant
					result.edges.push_back(e2);
					result.edges.push_back(e3);
				}
			}
		}
	}
}

void Helper::drawMeshOverMat(const Mesh &mesh, Mat &mat)
{
	Scalar lineColor(0, 0, 255); // red
	int thickness = 1;
	int linetype = 8;

	Point start, end;
	Quad q;
	for (unsigned int i = 0; i < mesh.quads.size(); i++)
	{
		q = mesh.quads.at(i);
		for (unsigned int j = 0; j < 4; j++)
		{
			if (j == 0)
			{
				start.x = q.v1.x;
				start.y = q.v1.y;
				end.x = q.v2.x;
				end.y = q.v2.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else if (j == 1)
			{
				start.x = q.v2.x;
				start.y = q.v2.y;
				end.x = q.v4.x;
				end.y = q.v4.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else if (j == 2)
			{
				start.x = q.v4.x;
				start.y = q.v4.y;
				end.x = q.v3.x;
				end.y = q.v3.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else
			{
				start.x = q.v3.x;
				start.y = q.v3.y;
				end.x = q.v1.x;
				end.y = q.v1.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
		}
	}
}

double Helper::stringToDouble(const string &s)
{
	istringstream i(s);
	double x;
	if (!(i >> x))
		return 0;
   return x;
}

Mat Helper::meshAsMat(const Mesh &mesh, const Size &s)
{
	Mat mat = Mat::zeros(s.height, s.width, CV_8UC3);
	
	// line parameters
	Scalar lineColor(0, 0, 0); // black
	int thickness = 1;
	int linetype = 8;

	// make everything white
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			mat.at<Vec3b> (y, x) [0] = 255;
			mat.at<Vec3b> (y, x) [1] = 255;
			mat.at<Vec3b> (y, x) [2] = 255;
		}
	}

	// draw lines
	Point start, end;
	Quad q;
	for (unsigned int i = 0; i < mesh.quads.size(); i++)
	{
		q = mesh.quads.at(i);
		for (unsigned int j = 0; j < 4; j++)
		{
			if (j == 0)
			{
				start.x = q.v1.x;
				start.y = q.v1.y;
				end.x = q.v2.x;
				end.y = q.v2.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else if (j == 1)
			{
				start.x = q.v2.x;
				start.y = q.v2.y;
				end.x = q.v4.x;
				end.y = q.v4.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else if (j == 2)
			{
				start.x = q.v4.x;
				start.y = q.v4.y;
				end.x = q.v3.x;
				end.y = q.v3.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else
			{
				start.x = q.v3.x;
				start.y = q.v3.y;
				end.x = q.v1.x;
				end.y = q.v1.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
		}
	}

	return mat;
}

void Helper::matXmat(const Mat &a, const Mat &b, Mat &dest)
{
	Size destSize(a.cols, a.rows);
	int atype = a.type();
	dest = Mat::zeros(destSize, CV_32F);
	
	cv::normalize(a, a, 0, 1, NORM_MINMAX);

	for (int y = 0; y < a.rows; y++)
	{
		for (int x = 0; x < a.cols; x++)
		{
			dest.at<float> (y, x) = a.at<float> (y, x) * b.at<uchar> (y, x);
		}
	}

	a.convertTo(a, CV_32F, 255);
	dest.convertTo(dest, b.type());
}

void Helper::getImageROI(Quad &quad, Mat &roi, Mat &img)
{
	int roiX, roiY, roiWidth, roiHeight;
	Vertex topleft, topright, bottomleft;

	if (quad.v1.y > quad.v2.y)
	{
		roiY = quad.v2.y;
		topright.y = quad.v2.y;
	}
	else
	{
		roiY = quad.v1.y;
		topright.y = quad.v1.y;
	}

	if (quad.v1.x > quad.v3.x)
	{
		roiX = quad.v3.x;
		bottomleft.x = quad.v3.x;
	}
	else
	{
		roiX = quad.v1.x;
		bottomleft.x = quad.v1.x;
	}

	if (quad.v2.x < quad.v4.x)
		topright.x = quad.v4.x;
	else
		topright.x = quad.v2.x;

	if (quad.v3.y < quad.v4.y)
		bottomleft.y = quad.v4.y;
	else
		bottomleft.y = quad.v3.y;

	topleft.y = roiY;
	topleft.x = roiX;

	roiWidth = (int) WarpingMath::getDistance(topleft, topright);
	roiHeight = (int) WarpingMath::getDistance(topleft, bottomleft);

	roi = img(Rect(roiX, roiY, roiWidth, roiHeight));
}

Quad Helper::getRelativeCoordinates(Quad &quad)
{
	Vertex topleft;

	if (quad.v1.y > quad.v2.y)
		topleft.y = quad.v2.y;
	else
		topleft.y = quad.v1.y;

	if (quad.v1.x > quad.v3.x)
		topleft.x = quad.v3.x;
	else
		topleft.x = quad.v1.x;

	Quad result;

	result.v1 = quad.v1 - topleft;
	result.v2 = quad.v2 - topleft;
	result.v3 = quad.v3 - topleft;
	result.v4 = quad.v4 - topleft;

	return result;
}