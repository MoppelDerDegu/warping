#include "Helper.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

IplImage* getNthFrame(CvCapture* capture, int n)
{
	for(int i = 0; i <= n; i++)
	{
		if(cvQueryFrame(capture) == NULL)
			return NULL;
	}

	return cvQueryFrame(capture);
}
