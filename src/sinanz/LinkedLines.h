/*
 * LinkedLines.h
 *
 *  Created on: Nov 13, 2011
 *      Author: root
 */

#ifndef LINKEDLINES_H_
#define LINKEDLINES_H_

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class LinkedLines {
public:
	int distanceBetweenLines;
	cv::vector<cv::vector<cv::Vec4f > > rawLines;
	cv::Mat filteredLinesImage;
	int plottingLineSize;

	/**
	 * connectedLines are lines satisfying the connectivity condition (lines close to each other).
	 */
	cv::vector<cv::vector<cv::Vec4f > > connectedLines;
	cv::vector<cv::vector<cv::Vec4f > > linkedLines;

	LinkedLines(cv::vector<cv::vector<cv::Vec4f> > rlines,
			int g_distance_between_lines,
			int g_angle_threshold_ratio,
			int g_line_size_for_plotting,
			cv::Mat filteredLinesImage);
	virtual ~LinkedLines();

	// Getters and Setters
	cv::vector<cv::vector<cv::Vec4f> > getRawLines();
	cv::vector<cv::vector<cv::Vec4f> > getConnectedLines();
	cv::vector<cv::vector<cv::Vec4f> > getLinkedLines();
	void setRawLines(cv::vector<cv::vector<cv::Vec4f> > rLines);
	void setConnectedLines(cv::vector<cv::vector<cv::Vec4f> > cLines);
	void setLinkedLines(cv::vector<cv::vector<cv::Vec4f> > lLines);

	/**
	 * A method that will connect the rawlines into connectedlines by satisfying the following connectivity condition:
	 * A line is connected to another line if the max. distance between both is less than a threshold distance.
	 */
	void connectRawLines(cv::vector<cv::vector<cv::Vec4f> > rLines, int maxDistance);
	/**
	 * A function to calculate the euclidean distance between two points. Used in connectRawLines().
	 */
	double findDistanceBetweenPoints(cv::Point pt1, cv::Point pt2);
	cv::Mat plotLines(cv::vector<cv::vector<cv::Vec4f> > lines1, int lineSize, cv::Mat output_image);
	cv::Mat plot1Line(cv::Vec4f current_line, int lineSize, cv::Scalar color, cv::Mat output_image);
	cv::vector<cv::vector<float> > calculateAnglesOfConnectedLines(cv::vector<cv::vector<cv::Vec4f> > cLines);
	/**
	 * A function that will link the connected lines, by satisfying the curvature condition.
	 * This function uses the calculateAnglesOfConnectedLines() and anglesHaveSameSign functions.
	 */
	void linkConnectedLines(cv::vector<cv::vector<cv::Vec4f> > cLines, double thresholdRatio);

	/*
	 * A function that takes a set of lines (a contour) and checks if the angles of the lines have the same sign.
	 * This function is used in linkConnectedLines to fulfill the curvature condition.
	 */
	bool anglesHaveSameSign(cv::vector<float> anglesOfOneContour, double thresholdRatio);

	/*
	 * A function that checks the difference in angles for successive lines in a contour. Needed for the connectivity criteria.
	 */
	bool anglesDiffAcceptable(cv::vector<float> anglesOfOneContour);
	void plot();
};

#endif /* LINKEDLINES_H_ */
