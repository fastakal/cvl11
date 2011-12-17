/*
 * LinkedLines.cpp
 *
 *  Created on: Nov 13, 2011
 *      Author: root
 */

#include "LinkedLines.h"

LinkedLines::LinkedLines(cv::vector<cv::vector<cv::Vec4f> > rLines,
		int g_distance_between_lines,
		int g_angle_threshold_ratio,
		int g_line_size_for_plotting,
		cv::Mat filteredLinesImg) {

	setRawLines(rLines);
	distanceBetweenLines = g_distance_between_lines;
	filteredLinesImage = filteredLinesImg;
	plottingLineSize = g_line_size_for_plotting;

	connectRawLines(rawLines, distanceBetweenLines);
	linkConnectedLines(connectedLines, g_angle_threshold_ratio);
}

void LinkedLines::plot(){
	filteredLinesImage = plotLines(linkedLines, plottingLineSize, filteredLinesImage);
}

LinkedLines::~LinkedLines() {
	// TODO Auto-generated destructor stub
}

// Getters and Setters
cv::vector<cv::vector<cv::Vec4f> > LinkedLines::getRawLines(){
	return rawLines;
}

cv::vector<cv::vector<cv::Vec4f> > LinkedLines::getConnectedLines(){
	return connectedLines;
}

cv::vector<cv::vector<cv::Vec4f> > LinkedLines::getLinkedLines(){
	return linkedLines;
}

void LinkedLines::setRawLines(cv::vector<cv::vector<cv::Vec4f> > rLines){
	rawLines = rLines;
}

void LinkedLines::setConnectedLines(cv::vector<cv::vector<cv::Vec4f> > cLines){
	connectedLines = cLines;
}

void LinkedLines::setLinkedLines(cv::vector<cv::vector<cv::Vec4f> > lLines){
	linkedLines = lLines;
}

/**
 * A method that will connect the rawlines into connectedlines by satisfying the following connectivity condition:
 * A line is connected to another line if the max. distance between both is less than a threshold distance.
 */
void LinkedLines::connectRawLines(cv::vector<cv::vector<cv::Vec4f> > rLines, int maxDistance){

	int noOfLineSets = rLines.size();
	cv::vector<cv::vector<cv::Vec4f> > cLines; cLines.resize(noOfLineSets);
	cv::vector<cv::Vec4f > currentLineSet;

	cv::Point point1, point2;
	double currentDistance;

	for( int i = 0; i < noOfLineSets; i++){
		currentLineSet = rLines[i];
		int noOfLines = currentLineSet.size();
		cv::vector<cv::Vec4f> selectedLineSet; selectedLineSet.resize(noOfLines);
		int counter = 0;

		for( int j = 0; j < noOfLines-1; j++){
			point1 = cv::Point(currentLineSet[j][2], currentLineSet[j][3]);
			point2 = cv::Point(currentLineSet[j+1][2], currentLineSet[j+1][3]);
			currentDistance = findDistanceBetweenPoints(point1, point2);

			if( (int) currentDistance < maxDistance){
				selectedLineSet[counter] = currentLineSet[j];
				counter++;
			}
		}
		selectedLineSet.resize(counter);
		cLines[i] = selectedLineSet;
	}
	connectedLines = cLines;
}

/**
 * A function to calculate the euclidean distance between two points. Used in connectRawLines().
 */
double LinkedLines::findDistanceBetweenPoints(cv::Point pt1, cv::Point pt2){
	double xDistance = pt1.x - pt2.x;
	double yDistance = pt1.y - pt2.y;
	double distance = sqrt(xDistance*xDistance + yDistance*yDistance);
	return distance;
}

/**
 * A function that plots the lines.
 */
cv::Mat LinkedLines::plotLines(cv::vector<cv::vector<cv::Vec4f>> lines1, int lineSize, cv::Mat output_image){

	int numberOfContours = lines1.size();
	for(int i = 0; i < numberOfContours; i++){
		cv::vector<cv::Vec4f> currentContour = lines1[i];
		int numberOfLines = currentContour.size();
		cv::Scalar color = cv::Scalar(rand() * 255, rand() * 255, rand() * 255);
		for( int j = 0; j < numberOfLines; j++ ){
			output_image = plot1Line(currentContour[j], lineSize, color, output_image);
		}
	}
	return output_image;
}

/**
 * A function that plots one line, with an input size, to an output image. Used by plotLines().
 */
cv::Mat LinkedLines::plot1Line(cv::Vec4f current_line, int lineSize, cv::Scalar color, cv::Mat output_image){

	int d;
	cv::Point startPoint, endPoint;

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

/**
 * A function that calculates the angle (slope) of a line for a cv::vector of lines.
 */
cv::vector<cv::vector<float> > LinkedLines::calculateAnglesOfConnectedLines(cv::vector<cv::vector<cv::Vec4f>> cLines){
	int noSetOfLines = cLines.size();
	cv::vector<cv::vector<float> > angles; angles.resize(noSetOfLines);
	cv::vector<cv::Vec4f> setOfLines;
	cv::vector<float> setOfAngles;
	int noOfLines;

	for( int i = 0; i < noSetOfLines; i++){
		setOfLines = cLines[i];
		noOfLines = setOfLines.size();
		setOfAngles.resize(noOfLines);

		for( int j = 0; j < noOfLines-1; j++){
			cv::Vec4f currentLine = setOfLines[j];
			cv::Vec4f nextLine = setOfLines[j+1];
			float x1 = currentLine[0]; float y1 = currentLine[1];
			float x2 = nextLine[0]; float y2 = nextLine[1];
			float angle = atan((y2 - y1)/(x2 - x1));
			setOfAngles[j] = angle;
		}
		angles[i] = setOfAngles;
	}
	return angles;
}

/**
 * A function that will link the connected lines, by satisfying the curvature condition.
 */
void LinkedLines::linkConnectedLines(cv::vector<cv::vector<cv::Vec4f> > cLines, double thresholdRatio){

	cv::vector<cv::vector<float> > angles;
	angles = calculateAnglesOfConnectedLines(cLines);
	cv::vector<cv::vector<cv::Vec4f> > lLines; lLines.resize(cLines.size());
	int noSetOfLines = cLines.size();
	cv::vector<cv::Vec4f> setOfLines;
	int noOfLines;
	cv::vector<float> anglesOfOneContour;
	bool isAnglesHaveTheSameSign;
	bool isDifferenceInAngleAcceptable;
	int validContoursContour = 0;

	for( int i = 0; i < noSetOfLines; i++){
		setOfLines = cLines[i];
		noOfLines = setOfLines.size();
		anglesOfOneContour = angles[i];
		isAnglesHaveTheSameSign = anglesHaveSameSign(anglesOfOneContour, thresholdRatio);
		isDifferenceInAngleAcceptable = anglesDiffAcceptable(anglesOfOneContour);


		if (isAnglesHaveTheSameSign && isDifferenceInAngleAcceptable){
			lLines[validContoursContour] = setOfLines;
			validContoursContour++;
		}
	}
	lLines.resize(validContoursContour);
	setLinkedLines(lLines);
}

bool LinkedLines::anglesHaveSameSign(cv::vector<float> anglesOfOneContour, double thresholdRatio){
	int size = anglesOfOneContour.size();
	double positiveAnglesCounter = 0;
	double negativeAnglesCounter = 0;
	double totalCounter;

	for( int i = 0; i < size; i++){

		if (anglesOfOneContour[i] > 0) {
			positiveAnglesCounter++;
			totalCounter++;
		} else {
			negativeAnglesCounter++;
			totalCounter++;
		}
	}
	bool answer = ((((positiveAnglesCounter)/totalCounter)> thresholdRatio) ||
			((negativeAnglesCounter)/totalCounter)> thresholdRatio);

	return answer;
}
bool LinkedLines::anglesDiffAcceptable(cv::vector<float> anglesOfOneContour){

	int size = anglesOfOneContour.size();
	float currentAngle, nextAngle;
	int diffInAngles;
	int zeroCounter = 0;

	for( int i = 0; i < size - 1; i++){
		currentAngle = anglesOfOneContour[i];
		nextAngle = anglesOfOneContour[i+1];
		diffInAngles = abs((int)((currentAngle - nextAngle)/currentAngle));
		if (diffInAngles==0)
			zeroCounter++;
	}

	double ratio = (double)zeroCounter / (double)size;
	return ratio > 0.7;
}
