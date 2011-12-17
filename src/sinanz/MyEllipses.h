/*
 * MyEllipses.h
 *
 *  Created on: Nov 18, 2011
 *      Author: root
 */

#ifndef MyEllipses_H_
#define MyEllipses_H_
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class MyEllipses {
public:
	MyEllipses(cv::vector<cv::vector<cv::Vec4f>> lLines, cv::Mat eImage);
	virtual ~MyEllipses();

	cv::Mat edgeImage;
	cv::Mat ellipsesImage;
	cv::Mat edgeImagePlusEllipses;

	cv::vector<cv::vector<cv::Vec4f>> linkedLines;
	cv::vector<cv::RotatedRect> fittedEllipses;
	cv::vector<cv::vector<cv::Point> > linkedPoints;

	cv::vector<cv::RotatedRect> filteredEllipses;
	cv::vector<cv::vector<cv::Point> > filteredPoints;
	cv::RotatedRect chosenEllipse;

	/**
	 * This function simply takes one point out of each line. Thus returning a vector of vector of points. This is needed
	 * for fitting the ellipses as we need points not lines.
	 */
	void ExtractPointsFromLines();

	/**
	 * This function fits ellipses to each set of points (contour) and plots them as well to ellipsesImage.
	 */
	void fitEllipses();

	/**
	 * A function that takes the fittedEllipses variable, and filters them according to the following criterias:
	 * A potential hoop ellipse should have a ratio between axes that is bigger than 0.5.
	 *
	 */
	void eliminateEllipses();

	/**
	 * A function that validates an ellipse according to the criteria in eliminateEllipses(), used by eliminateEllilpses().
	 */
	bool validateEllipse(cv::RotatedRect anEllipse);

	/**
	 * Another function to validate ellipses. Based on having an error measure of
	 * points of contours with respect to their respective position
	 * on the fitted ellipse.
	 */
	bool errorMeasureEllipse(cv::RotatedRect anEllipse, cv::vector<cv::Point> setOfPoints);

	/**
	 * A function that will take the edge image and plot the ellipses on it.
	 */
	void plotEllipsesOnEdgeImage();

	/**
	 * A function that will take the edge image and the ellipses, then will calculate
	 * how many edge points lie on the ellipses in respect to the whole tested points.
	 */
	void rejectEllipsesFromEdgeImage();

	/**
	 * A function that takes a position (x and y) and returns 1 if the value in that neighbourhood is 255, 0 otherwise.
	 */
	int getPointsValue(int x,int y);

	cv::RotatedRect getBestEllipse(cv::vector<cv::RotatedRect> filtEl, cv::vector<double> qualityOfEllipses);
	void plotChosenEllipseOnEdgeImage();
	void plot(cv::Mat lImage);

};

#endif /* MyEllipses_H_ */
