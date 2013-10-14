#include "Helper.h"
#include "stdafx.h"
#include "WarpingMath.h"
#include "FileManager.h"

IplImage* Helper::getNthFrame(CvCapture* capture, int n)
{
	for (int i = 0; i < n - 1; i++)
	{
		if (cvQueryFrame(capture) == NULL)
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

void Helper::drawMeshesOverMatStereo(Mesh &leftMesh, Mesh &rightMesh, Mat &mat)
{
	Scalar lineColor(0, 0, 255); // red
	int thickness = 2;
	int linetype = 8;

	Point start, end;
	Quad q;
	Mat roi;
	
	//left
	roi = mat(Rect(0, 0, mat.size().width / 2, mat.size().height));

	for (unsigned int i = 0; i < leftMesh.quads.size(); i++)
	{
		q = leftMesh.quads.at(i);

		start.x = q.v1.x;
		start.y = q.v1.y;
		end.x = q.v2.x;
		end.y = q.v2.y;
		line(roi, start, end, lineColor, thickness, linetype);

		start.x = q.v2.x;
		start.y = q.v2.y;
		end.x = q.v4.x;
		end.y = q.v4.y;
		line(roi, start, end, lineColor, thickness, linetype);

		start.x = q.v4.x;
		start.y = q.v4.y;
		end.x = q.v3.x;
		end.y = q.v3.y;
		line(roi, start, end, lineColor, thickness, linetype);

		start.x = q.v3.x;
		start.y = q.v3.y;
		end.x = q.v1.x;
		end.y = q.v1.y;
		line(roi, start, end, lineColor, thickness, linetype);
	}
	
	int lel = mat.size().width / 2;

	//right
	roi = mat(Rect(mat.size().width / 2, 0, mat.size().width / 2, mat.size().height));

	for (unsigned int i = 0; i < rightMesh.quads.size(); i++)
	{
		q = rightMesh.quads.at(i);

		start.x = q.v1.x;
		start.y = q.v1.y;
		end.x = q.v2.x;
		end.y = q.v2.y;
		line(roi, start, end, lineColor, thickness, linetype);

		start.x = q.v2.x;
		start.y = q.v2.y;
		end.x = q.v4.x;
		end.y = q.v4.y;
		line(roi, start, end, lineColor, thickness, linetype);

		start.x = q.v4.x;
		start.y = q.v4.y;
		end.x = q.v3.x;
		end.y = q.v3.y;
		line(roi, start, end, lineColor, thickness, linetype);

		start.x = q.v3.x;
		start.y = q.v3.y;
		end.x = q.v1.x;
		end.y = q.v1.y;
		line(roi, start, end, lineColor, thickness, linetype);
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

int Helper::stringToInt(const string &s)
{
	istringstream i(s);
	int x;

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

vector<string> &Helper::split(const string &s, char delimiter, vector<string> &result)
{    
	stringstream ss(s);
    string item;

    while (getline(ss, item, delimiter)) {
        result.push_back(item);
    }

    return result;
}

vector<string> Helper::split(const string &s, char delimiter)
{
	vector<string> result;
	split(s, delimiter, result);
	return result;
}

void Helper::adjustRightPathlineCoordinates(PathlineSets &rightPathlines, Size &newImageSize)
{
	for (unsigned int i = 0; i < rightPathlines.pathlines.size(); i++)
	{
		for (unsigned int j = 0; j < rightPathlines.pathlines.at(i).size(); j++)
		{
			for (unsigned int k = 0; k < rightPathlines.pathlines.at(i).at(j).path.size(); k++)
			{
				rightPathlines.pathlines.at(i).at(j).path.at(k).second.x = rightPathlines.pathlines.at(i).at(j).path.at(k).second.x - (newImageSize.width / 2.0);
			}
		}
	}
}

void Helper::printUsageInstructions()
{
	cout << "USAGE: WarpingStereoVideo.exe inputFileName interpolatedOutputFileName finalOutputFileName numberOfWarpingEveryNthFrame newWidth newHeight" << endl;
}