/*
 * SegmentedDepthImage.h
 *
 *  Created on: Nov 20, 2011
 *      Author: root
 */

#ifndef SegmentedDepthImage_H_
#define SegmentedDepthImage_H_

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class SegmentedDepthImage {
	cv::Mat depthImage;
	cv::Mat filteredDepthImage;
	cv::Mat edgeDepthImage;
public:
	SegmentedDepthImage(cv::Mat input, int cannyMin, int cannyMax, int dilate);
	virtual ~SegmentedDepthImage();
	/**
	 * Pre_process function:
	 * A function that does the pre-processing stage on the input image.
	 * For the moment, simple Guassian blurring is done only.
	 */
	void pre_process(cv::Mat input_img);

	/**
	 * A function that segments the input image by using canny edge.
	 */
	void segmentDepthImage(int cannyMin, int cannyMax, int dilate);
	cv::Mat getEdgeImage();
};

#endif /* SegmentedDepthImage_H_ */
