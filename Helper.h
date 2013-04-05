#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>

IplImage* getNthFrame(CvCapture* capture, int n);

