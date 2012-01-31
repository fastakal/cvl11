/*
 * segmentedImage.h
 *
 *  Created on: Nov 11, 2011
 *      Author: root
 */

#ifndef SEGMENTEDIMAGE_H_
#define SEGMENTEDIMAGE_H_

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

class segmentedImage {
	Mat input_image;
	Mat filtered_image;
	Mat edgeImage;
public:
	segmentedImage (Mat input, int cannyMin, int cannyMax);
	~segmentedImage ();
	Mat getEdgeImage();

	/**
	 * Pre_process function:
	 * A function that does the pre-processing stage on the input image.
	 * For the moment, simple Guassian blurring is done only.
	 */
	void pre_process(Mat input_img);

	/**
	 * A function that segments the input image by using canny edge detector by opencv.
	 */
	void segmentThisImage(int cannyMin, int cannyMax);
};

#endif /* SEGMENTEDIMAGE_H_ */
