#include "Helper.h"
#include "stdafx.h"
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

float Helper::getDistance(Vertex v1, Vertex v2)
{
	return sqrt(static_cast<float>(sqr(v1.x - v2.x) + sqr(v1.y - v2.y)));
}

float Helper::getAverageSaliency(int sumOfSaliencyValues, int numOfPixel)
{
	float res = ((float) sumOfSaliencyValues) / ((float) numOfPixel);

	if (res < 0)
		return 0;
	else
		return res;
}

float Helper::normalize(float value, float max)
{
	float res = value / max;
	
	if (res < 0)
		return 0;
	else
		return res;
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

double Helper::euclideanNorm(const Vertex &v)
{
	return sqrt((double) (sqr(v.x) + sqr(v.y)));
}

void Helper::saveGrid(const string fileName, const string dir, const Mesh &m, const Size &s)
{
	const char* _dir = (char*) dir.c_str();
	FileManager::mkDir(_dir);

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
	for (unsigned int i = 0; i < m.quads.size(); i++)
	{
		q = m.quads.at(i);
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

	Point center;
	for (unsigned int i = 0; i < m.vertices.size(); i++)
	{
		center.x = m.vertices.at(i).x;
		center.y = m.vertices.at(i).y;

		circle(mat, center, 3, Scalar(0, 0, 255), -1, 8);
	}

	imwrite(dir + fileName, mat);
}