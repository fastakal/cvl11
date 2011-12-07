/*
 * SegmentedDepthImage.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: root
 */

#include "SegmentedDepthImage.h"

SegmentedDepthImage::SegmentedDepthImage(cv::Mat input, int cannyMin, int cannyMax, int dilate) {
	depthImage = input;
	pre_process(depthImage);
	segmentDepthImage(cannyMin, cannyMax, dilate);
}

SegmentedDepthImage::~SegmentedDepthImage() {
	// TODO Auto-generated destructor stub
}

/**
 * Pre_process function:
 * A function that does the pre-processing stage on the input image.
 * For the moment, simple Guassian blurring is done only.
 */
void SegmentedDepthImage::pre_process(cv::Mat input_img){
	cv::GaussianBlur(input_img, filteredDepthImage, cv::Size(3, 3), 1, 0, cv::BORDER_DEFAULT);
}

/**
 * A function that segments the input image by using canny edge.
 */
void SegmentedDepthImage::segmentDepthImage(int cannyMin, int cannyMax, int dilate){
	cv::Mat cnt_img = cv::Mat::zeros(480, 640, depthImage.type());

	// 2.1 find edges.

	cv::Canny(depthImage, cnt_img, cannyMin, cannyMax, 5);

	if(dilate == 1){
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 1));
		cv::dilate(cnt_img, cnt_img, kernel,cv::Point(-1,-1), 1);
	}
	edgeDepthImage = cnt_img;
}

cv::Mat SegmentedDepthImage::getEdgeImage(){
	return  edgeDepthImage;
}
