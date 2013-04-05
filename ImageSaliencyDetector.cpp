// author : Torben Dittrich
// edited by : Christopher Hipp


#include "Stdafx.h"
#include "ImageSaliencyDetector.h"


using namespace std;

ImageSaliencyDetector::ImageSaliencyDetector(void)
{
}


ImageSaliencyDetector::~ImageSaliencyDetector(void)
{
}

void ImageSaliencyDetector::quantify()
{
	cout << ">> quantify" << endl;

	// get references from mat object
	Mat &rImg = this -> rawImage;	
	Mat &tImg = this -> tmpImage;

	// declaration
	map<int, int> colors;				// first: color,		second: frequency/importance
	vector<pair<int, int>> rate;		// first: frequence,	second: color

	// constant variables
	const float drop = 0.046f; // 4.6% of the pixel will be replaced with similiar colors which are also a part of the image
	const int increase = 12; // decrease the color space from 255^3 to 12^3
	const int weight[3] = {pow((double) increase, 2) , pow((double) increase, 1), pow((double) increase, 0)};	// each channel needs another power of twelve, because e.g. 255-0-0 and 0-0-255 are not similiar

	// count the frequency of each color in the image
	for (int y = 0; y < rImg.rows; y++)
	{
		for (int x = 0; x < rImg.cols; x++)
		{			
			tImg.at<int>(y, x) = (int)(rImg.at<Vec3f>(y,x)[0]*(increase))*weight[0] + (int)(rImg.at<Vec3f>(y,x)[1]*(increase))*weight[1] + (int)(rImg.at<Vec3f>(y,x)[2]*(increase)); // similiar colors got the same number
			colors[tImg.at<int>(y, x)] += 1;	
		}
	}

	// transform color-map into a vector object
	rate.reserve(colors.size());
	for (map<int, int>::iterator itr = colors.begin(); itr != colors.end(); itr++) 
	{
		rate.push_back(pair<int, int>(itr->second, itr->first));
	}
	sort(rate.begin(), rate.end(), greater<pair<int, int>>()); // sort
	colors.clear();
	
	// resolve color value into three channels (basis is 12)
	vector<Vec3i> cPalatte(rate.size());
	for (unsigned int i = 0; i < rate.size(); i++)
	{
		int value = rate[i].second;
		cPalatte[i][0] = value / weight[0];
		cPalatte[i][1] = (value % weight[0]) / weight[1];
		cPalatte[i][2] = value % weight[1];
	}
	
	// detect the colors that should be removed
	int pDiff = rate.size();
	int reduce = cvRound(rImg.cols * rImg.rows * drop);					// number of pixels that will be replaced by another color
	for(int value = rate[pDiff-1].first; value < reduce; pDiff--)		// calculate how moch different colors will be removed
	{
			value += rate[pDiff-2].first;
			if(pDiff < 10)
			{
				pDiff = min((int) rate.size(), 100);
				break;
			}
	}
	
	// create map with the colors which should not be deleted 
	for (int i = 0; i < pDiff; i++)
	{
		colors[rate[i].second] = i; // i == pointer, that points to the real color
	}


	// calculate replacements for the colors that shoud be removed
	pair<int, int> valdex;	// index, value
	for (unsigned int i = pDiff; i < rate.size(); i++)
	{
		for (int j = 0; j < pDiff; j++)
		{
			int cDis = pow(cPalatte[i][0]-cPalatte[j][0], 2.0) + pow(cPalatte[i][1]-cPalatte[j][1], 2.0) + pow(cPalatte[i][2]-cPalatte[j][2], 2.0);	// compare colors - how similar they are
			if (j == 0 || cDis < valdex.second) {
				valdex = make_pair(j, cDis); // color index / distance
			}
		}
		colors[rate[i].second] = colors[rate[valdex.first].second]; // store the old color with a pointer to the replacement
	}

	// initialization of the result container
	this -> resImage = Mat::zeros(1, pDiff, CV_32FC3);
	this -> couImage = Mat::zeros(resImage.size(), CV_32S);

	// pointer 
	Vec3f* result = (Vec3f*) resImage.data;
	int* counter = (int*) couImage.data;

	// create image container with the remaining color values
	for (int y = 0; y < rImg.rows; y++) 
	{
		for (int x = 0; x < rImg.cols; x++)
		{			
			tImg.at<int>(y, x) = colors[tImg.at<int>(y, x)];	// replace the old color with new one
			
			result[tImg.at<int>(y, x)] += rImg.at<Vec3f>(y,x);	// create color palette
			counter[tImg.at<int>(y, x)] += 1;
		}
	}

	// calculate the average between the old colors with their replacement
	for (int i = 0; i < resImage.cols; i++)
	{
		result[i][0] /= counter[i];
		result[i][1] /= counter[i];
		result[i][2] /= counter[i];
	}

	cout << ">> colors: " << resImage.cols << endl;
	this->numColors = resImage.cols;

	// clean up
	colors.clear();
	rate.clear();
}

void ImageSaliencyDetector::measure()
{
	cout << ">> measure" << endl;

	// get references
	Mat &rImg = this -> resImage;
	Mat &cImg = this -> couImage;

	// initialization
	this -> disImage = Mat::zeros(1, rImg.cols, CV_32F);
	vector<vector<pair<float, int>>> col(rImg.cols);
	this->maxColorDistance = 0;

	// calculate the distance between each existing color
	for (int i = 0; i < rImg.cols; i++)			
	{	
		const Vec<float, 3> first = rImg.at<Vec3f>(0, i);		//choose a pixel and...
		for (int j = 0; j < rImg.cols; j++)
		{
			if (i == j) {
				col[i].push_back(make_pair(0.f, i));		// the distance to itself is 0
				continue;
			}
			const Vec<float, 3> second = rImg.at<Vec3f>(0, j); // ...compare it with all others

			float dis = sqrt(pow((first[0] - second[0]), 2)+pow((first[1] - second[1]), 2)+pow((first[2] - second[2]), 2));		// calculate distance (euklid)

			if(this->maxColorDistance < dis)								//store the maximum distance between colors
				this->maxColorDistance = dis;

			col[i].push_back(make_pair(dis, j));							// store distance and position of the compared color
			disImage.at<float>(0, i) += cImg.at<float>(0, j) * dis;			// calculate importance of each color (sum of weight*distance)
		}
		sort(col[i].begin(), col[i].end());	// sort pairs
	}

	// store resualt
	this -> collection = col;

	// clean up
	col.clear();
}

void ImageSaliencyDetector::smooth()
{
	cout << ">> smooth" << endl;

	// get references
	Mat &rImg = this -> resImage;
	Mat &dImg = this -> disImage;

	// constant variables
	const float diff = 4.0f;

	// initialization
	int smooth = max(cvRound((rImg.cols)/diff), 2);
	Mat tmp = Mat::zeros(1, rImg.cols, CV_32F);
	vector<float> distance(smooth, 0);
	vector<float> data(smooth);

	// check if there is more then one color
	if(dImg.cols <= 1) {
		return;
	}

	// calculate new weight (importance) based on the color distance
	for (int i = 0; i < rImg.cols; i++)		// for each color ...
	{
		float dSum = 0;
		float value = 0;

		data[0] = dImg.at<float>(0, i);
		for (int j = 1; j < smooth; j++)	
		{
			distance[j] = collection[i][j].first;						// store each distance to color x in vector distance 
			data[j] = dImg.at<float>(0, collection[i][j].second);		// store sum of (distance*gewicht) from x in vector data (see line 205)
			dSum += collection[i][j].first;								// sum of the distances of a color to all other	
		}

		// calculate new weight (smoothing weight)
		for (int j = 0; j < smooth; j++) 
		{
			value += data[j] * (dSum - distance[j]);			// refine the saliency value ...
		}
		tmp.at<float>(0, i) =  value / ((smooth-1) * dSum);		// ... of each color
	}

	// store resualt
	this -> disImage = tmp;

	// clean up
	distance.clear();
	data.clear();
}

void ImageSaliencyDetector::build() 
{
	cout << ">> build"  << endl;

	// initialization
	this -> resultHC =  Mat::zeros(rawImage.size(), CV_32F);

	// built Mat object
	for (int i = 0; i < rawImage.rows; i++)
	{
		for (int j = 0; j < rawImage.cols; j++)
		{
			resultHC.at<float>(i, j) = disImage.at<float>(0, tmpImage.at<int>(i,j)); // Wähle Farbe von Farbpalette
		}
	}

	// gaussian blur filter
	GaussianBlur(resultHC, resultHC, Size(3, 3), 0);
	
	// transform the values between 0...1
	normalize(resultHC, resultHC, 0, 1, NORM_MINMAX);
}

CvMat* ImageSaliencyDetector::hContrast(IplImage* img)
{
	cout << "\nStart Histogram Based Contrast" << endl;
	cout << "> load (image): " << "test" << endl;

	// load image
	Mat source = img;//imread(tPath, CV_LOAD_IMAGE_COLOR);
	source.convertTo(source, CV_32FC3, 1.0/255); // 0...255 -> 0...1
	
	// initialization
	this -> rawImage = source;
	this -> tmpImage = Mat::zeros(rawImage.size(), CV_32S);

	// quantizes the input image and create color palette
	quantify();

	// convert color palette from RGB to L*a*b* color space
	cvtColor(resImage, resImage, CV_BGR2Lab);
	
	// convert frequency of each color into percentage share
	normalize(couImage, couImage, 1, 0, NORM_L1, CV_32F);
	
	// calculate each color distance
	measure();

	// smoothing of the color space
	smooth();

	// build saliency map as Mat object
	build();

	resultHC = resultHC*255;
	
	this->saliencyMap = resultHC;
		
	//clean up
	source.~Mat();
	tmpImage.~Mat();
	resImage.~Mat();

	return &this->saliencyMap;
}

float ImageSaliencyDetector::getMaxColorDistance()
{
	return this->maxColorDistance;
}

int ImageSaliencyDetector::getNumColors()
{
	return this->numColors;
}