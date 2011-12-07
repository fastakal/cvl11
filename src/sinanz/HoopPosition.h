/*
 * HoopPosition.h
 *
 *  Created on: Nov 26, 2011
 *      Author: root
 */

#ifndef HOOPPOSITION_H_
#define HOOPPOSITION_H_

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class HoopPosition {
public:
	cv::Mat depthImage;
	cv::Mat disparityImage;
	cv::RotatedRect ellipse;
	cv::vector<cv::Vec3f> depthValuesOfHoop;
	cv::Vec3f plane;
	cv::vector<cv::Vec3f> selectedDepthPoints;
	cv::Mat disparityImageWithPlane;
	float colormap_jet[128][3];
	HoopPosition();
	HoopPosition(cv::Mat, cv::Mat dispImage, cv::RotatedRect el, int numberOfPointsForDepth);
	virtual ~HoopPosition();

	/**
	 * A function that will take the ellipse, find the depth values of numberOfPoints * points on its perimeter and save them to a vector of points.
	 *
	 */
	cv::vector<cv::Vec3f> findPointsValues(int numberOfPoints);

	/**
	 * A function that plots small circles in the position of the depth values, with a color that corresponds to the depth value.
	 */
	void plotDepthValues(cv::Mat& dispImage);

	/**
	 * A function that takes a position (x and y) and returns the value in that neighbourhood.
	 */
	cv::Vec3f get3DPoint(int x,int y);

	void fitPlane();

	void plotThe4Points();
	void addFittedPlaneToImage(cv::Mat& dispOrDepthImage);
	/**
	 * Used by addFittedPlaneToImage();
	 */
	cv::Mat rotatePlaneImage(cv::Mat& planeImage, double angle);

	/**
	 * A function that fills the plane image with depth values.
	 */
	cv::Mat fillPlaneImage(cv::Mat& planeImage);
};

#endif /* HOOPPOSITION_H_ */
