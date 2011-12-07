/*
 * segmentedImage.cc
 *
 *  Created on: Nov 11, 2011
 *      Author: root
 */

#include "segmentedImage.h"

segmentedImage::segmentedImage (Mat input, int cannyMin, int cannyMax, int dilate){
	input_image = input;
	pre_process(input_image);
	segmentThisImage(cannyMin, cannyMax, dilate);

}
segmentedImage::~segmentedImage(){}

/**
 * Getters
 */
Mat segmentedImage::getEdgeImage(){
	return edgeImage;
}
/**
 * Pre_process function:
 * A function that does the pre-processing stage on the input image.
 * For the moment, simple Guassian blurring is done only.
 */
void segmentedImage::pre_process(Mat input_img){
	cv::GaussianBlur(input_img, filtered_image, Size(3, 3), 1, 0, BORDER_DEFAULT);
}

/**
 * A function that segments the input image by using canny edge.
 */
void segmentedImage::segmentThisImage(int cannyMin, int cannyMax, int doDilate){
	Mat cnt_img = Mat::zeros(input_image.size(), CV_8UC3);

	// 2.1 find edges.
	cv::Canny(filtered_image, cnt_img, cannyMin, cannyMax, 5);

	if(doDilate == 1){
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 1));
		cv::dilate(cnt_img, cnt_img, kernel,Point(-1,-1), 1);
	}
	edgeImage = cnt_img;
}
