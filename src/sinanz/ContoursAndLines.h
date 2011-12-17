/*
 * ContoursAndLines.h
 *
 *  Created on: Nov 11, 2011
 *      Author: root
 */

#ifndef CONTOURSANDLINES_H_
#define CONTOURSANDLINES_H_

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CodeContainer.h"

using namespace cv;


class ContoursAndLines {
public:
	Mat edgeImage;
	Mat linesImage;
	vector<vector<Point> > contours;
	vector<vector<Point> > approximatedContours;
	Mat contoursImage;
	vector<vector<Vec4f>> lines;
	int lineSize;

	ContoursAndLines(Mat edge, int minLength, int maxLength, int approxOrder, int lineSize);
	virtual ~ContoursAndLines();
	vector<vector<Point> > getContours();
	void setContours(vector<vector<Point> > cont);
	Mat getContoursImage();
	void setContoursImage(Mat img);
	vector<vector<Vec4f>> getLines();
	void setLines(vector<vector<Vec4f>> l);

	void findContours();
	void approximateContour(vector<vector<Point>> input_contours, int minimumLength, int maximumLength, int approxOrder);
	void plotMyContours(vector<vector<Point>> contours);
	void decomposeContours(vector<vector<Point> > contours, int lineSize);
	void plot();
	void plotLines(vector<vector<Vec4f>> lines1, int lineSize, Mat output_image);
	Mat plot1Line(Vec4f line1, int lineSize, Scalar color, Mat output_image);
	double getTimeNow();
};

#endif /* CONTOURSANDLINES_H_ */
