/*
 * ContoursAndLines.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: root
 */

#include "ContoursAndLines.h"

ContoursAndLines::ContoursAndLines(Mat edge, int minLength, int maxLength, int approxOrder, int lineS) {
	double tempTime, time1, time2, time3, time4;
	tempTime = getTimeNow();
	linesImage = Mat::zeros(edge.size(), CV_8UC3);
	edgeImage = edge;
	lineSize = lineS;
	time1 = getTimeNow() - tempTime;
	tempTime = getTimeNow();
	findContours();
	time2 = getTimeNow() - tempTime;
	tempTime = getTimeNow();
	approximateContour(contours, minLength, maxLength, approxOrder);
	time3 = getTimeNow() - tempTime;
	tempTime = getTimeNow();
	// If the size of line is 0, we fix it to 1 so that it doesn't crash.
	if (lineSize == 0){
		lineSize = 1;
	}
	decomposeContours(approximatedContours, lineSize);
	time4 = getTimeNow() - tempTime;

	printf("ContoursAndLines: Init: %f, finContours: %f, approximateContours: %f, decomposeContours: %f", time1, time2, time3, time4);

}

double ContoursAndLines::getTimeNow(){
	struct 	timeval tp;
	double sec, usec, time;

	gettimeofday(&tp, NULL);
	sec = static_cast<double>( tp.tv_sec);
	usec = static_cast<double>( tp.tv_usec) / 1E6;
	time = sec + usec;

	return time;
}


void ContoursAndLines::plot(){
	plotMyContours(approximatedContours);
	plotLines(lines, lineSize, linesImage);
}

ContoursAndLines::~ContoursAndLines() {
	// TODO Auto-generated destructor stub
}

Mat ContoursAndLines::getContoursImage(){
	return contoursImage;
}
void ContoursAndLines::setContoursImage(Mat img){
	contoursImage = img;
}

/**
 * A function that gets the contours from an edge image using the opencv function findContours with predefined parameters.
 */
void ContoursAndLines::findContours() {
	std::vector<vector<Point> > cont; Mat temp; edgeImage.copyTo(temp);
	cv::findContours(temp, cont, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	setContours(cont);
}
vector<vector<Point> > ContoursAndLines::getContours(){
	return contours;
}

void ContoursAndLines::setContours(vector<vector<Point> > cont){
	contours = cont;
}

vector<vector<Vec4f>> ContoursAndLines::getLines(){
	return lines;
}

void ContoursAndLines::setLines(vector<vector<Vec4f>> l){
	lines = l;
}

/**
 * A function that approximates the contours using the opencv function approximatePolyDP and minimum and maximum length of the contours.
 */
void ContoursAndLines::approximateContour(vector<vector<Point>> input_contours, int minimumLength, int maximumLength, int approxOrder){
	vector<vector<Point>> contours1;
	int j = 0;
	if (input_contours.size()){
		contours1.resize(input_contours.size());

		// Approximate the contours by a polynomial if their size is bigger than the minimumLength, otherwise, they're zeros.
		for( size_t k = 0; k < input_contours.size(); k++ ) {
			if (input_contours[k].size()>minimumLength &&
					input_contours[k].size() < maximumLength){
				//cv::approxPolyDP(Mat(input_contours[k]), contours1[j], approxOrder, false);
				contours1[j] = input_contours[k];
				j+=1;
			}
		}
		contours1.resize(j);

	}
	approximatedContours = contours1;
}

/**
 * A function to create an image containing the contours.
 */
void ContoursAndLines::plotMyContours(vector<vector<Point>> contours) {
	Mat output_image = Mat::zeros(edgeImage.size(), CV_8UC3);
	Scalar color(255, 255, 255);

	if (contours.size()){
		// Drawing all the contours
		drawContours( output_image, contours, -1, color, 0, 8 );
	}
	setContoursImage(output_image);
}

/**
 * A function that cuts contours into series of contours of small (but large enough) sizes. This function is needed before calling
 * findLines function, so that we approximate correctly the contours (to avoid approximating half of the ellipse by one single line).
 */
void ContoursAndLines::decomposeContours(vector<vector<Point> > contours, int lineSize){

	// output is a vector of vector of lines. a
	// vector of contours, where each contour is represented by a vector of lines.
	double tempTime, time1, time2, insideTime, time4;
	tempTime = getTimeNow();

	int totalNumberOfContours = contours.size();
	int contourSize;
	int numberOfLines;
	vector<vector<Vec4f>> output_lines;
	output_lines.resize(contours.size());

	time1 = getTimeNow() - tempTime;
	tempTime = getTimeNow();
	for( int i = 0; i < totalNumberOfContours; i++){
		contourSize = contours[i].size();
		numberOfLines = (int) ((double)contourSize/(double)lineSize);
		vector<Point> currentLine;
		currentLine.resize(lineSize);
		Vec4f this_line;
		vector<Vec4f> current_contour;
		current_contour.resize(numberOfLines);
		insideTime = getTimeNow();
		for( int j = 0; j < numberOfLines; j++ ){
			for( int k = 0; k < lineSize; k++ ){
				currentLine[k] = contours[i].at(j*lineSize + k);
			}
			fitLine(Mat(currentLine), this_line, CV_DIST_L1, 0, 0.01,0.01);
			current_contour[j] = this_line;
		}
		time4 = getTimeNow() - insideTime;
		output_lines[i] = current_contour;
	}
	time2 = getTimeNow() - tempTime;
	setLines(output_lines);
	printf("\nDecomposeMethod: init: %f, mainLoop: %f, insideTime: %f\n", time1, time2, time4);
}


/**
 * A function that plots the lines.
 */
void ContoursAndLines::plotLines(vector<vector<Vec4f>> lines1, int lineSize, Mat output_image){

	int numberOfContours = lines1.size();
	for(int i = 0; i < numberOfContours; i++){
		vector<Vec4f> currentContour = lines1[i];
		int numberOfLines = currentContour.size();
		Scalar color = Scalar(rand() * 255, rand() * 255, rand() * 255);
		for( int j = 0; j < numberOfLines; j++ ){
			output_image = plot1Line(currentContour[j], lineSize, color, output_image);
		}
	}
	linesImage = output_image;
}

/**
 * A function that plots one line, with an input size, to an output image. Used by plotLines().
 */
Mat ContoursAndLines::plot1Line(Vec4f current_line, int lineSize, Scalar color, Mat output_image){

	int d;
	Point startPoint, endPoint;

	d = sqrt((double)current_line[0]*current_line[0] + (double)current_line[1]*current_line[1]);

	current_line[0] = current_line[0] / d;
	current_line[1] = current_line[1] / d;

	startPoint.x = round(current_line[2] - current_line[0]*lineSize);
	startPoint.y = round(current_line[3] - current_line[1]*lineSize);
	endPoint.x = round(current_line[2] + current_line[0]*lineSize);
	endPoint.y = round(current_line[3] + current_line[1]*lineSize);
	line(output_image, startPoint, endPoint, color, 1, 1);

	return output_image;
}
