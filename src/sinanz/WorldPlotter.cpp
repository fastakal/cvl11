/*
 * WorldPlotter.cpp
 *
 *  Created on: Jan 5, 2012
 *      Author: dushan / sinan
 */

#include "WorldPlotter.h"

WorldPlotter::WorldPlotter() {

	// Initialization Phase.
	plot_size_x = 800;
	plot_size_y = 600;

	real_size_x = 10;
	real_size_y = 10;
  marker_size = 5;	
	x_color = Scalar(255/2, 255/2, 255/2);
	y_color = Scalar(255/2, 255/2, 255/2);
	normal_color = Scalar(0, 0, 255);
	normal_thickness = 2;
	object_size = 4;
	object_color = Scalar(128, 128, 0);
	object_thickness = 3;
	quad_color = Scalar(0, 128, 128);
	text_color = Scalar(255, 255, 255);
	font_scale = 0.7;
}

WorldPlotter::~WorldPlotter() {}

/**
 * This is the function that plots the helicopter AND the object of interest, along with its normal.
 */
void WorldPlotter::plotTopView(
		Point3f objectPosition,
		Point3f objectNormal,
		Point3f quadPosition,
		Point3f quadOrientation){

	Mat plot = Mat::zeros(plot_size_y, plot_size_x, CV_8UC3);
  plotAxes(plot);
	
	// Plot Normal Vector
	Point2i object_normal_p1, object_normal_p2;

	object_normal_p1.x = objectPosition.x / real_size_x * plot_size_x
			+ plot_size_x / 2;
	object_normal_p1.y = objectPosition.y / real_size_y * plot_size_y
			+ plot_size_y / 2;

	object_normal_p2.x = object_normal_p1.x - 25 * objectNormal.x;
	object_normal_p2.y = object_normal_p1.y - 25 * objectNormal.y;

	// Keep track, to plot the trace of the object.
	object_trace.push_back(object_normal_p1);
	plotTrace(plot, object_trace, object_color);

	line(plot, object_normal_p1, object_normal_p2, normal_color, normal_thickness);

	// Plot the object (as as small rectangle).
	rectangle(plot,
			Point2i(object_normal_p1.x - object_size, object_normal_p1.y - object_size),
			Point2i(object_normal_p1.x + object_size, object_normal_p1.y + object_size),
			object_color,
			object_thickness);

	float x, y, z;
	x = quadPosition.x;
	y = quadPosition.y;
	z = quadPosition.z;

	float roll, pitch, yaw;
	roll  = quadOrientation.x;
	pitch = quadOrientation.y;
	yaw   = quadOrientation.z;

	float q_x, q_y;

	q_x = (x + 0.1 * cos(yaw)) / real_size_x * plot_size_x
			+ plot_size_x / 2;
	q_y = (y + 0.1* sin(yaw)) / real_size_y * plot_size_y
			+ plot_size_y / 2;

	x = x / real_size_x * plot_size_x + plot_size_x / 2;
	y = y / real_size_y * plot_size_y + plot_size_y / 2;

	// Track the quad, plot the trace.
	quad_trace.push_back(Point2i(x, y));
	plotTrace(plot, quad_trace, quad_color);

	rectangle(plot,
			Point2i(x - object_size, y - object_size),
			Point2i(x + object_size, y + object_size),
			quad_color, object_thickness);

	rectangle(plot,
			Point2i(q_x - 1, q_y - 1),
			Point2i(q_x + 1, q_y + 1),
			quad_color, object_thickness);

	line(plot,
			Point2i(x, y),
			Point2i(q_x, q_y),
			normal_color,
			normal_thickness);

	putText(plot,
			"Object",
			Point2i(object_normal_p1.x, object_normal_p1.y - 10),
			FONT_HERSHEY_PLAIN,
			1,
			text_color);

	putText(plot,
			"Quad",
			Point2i(x, y - 10),
			FONT_HERSHEY_PLAIN,
			1,
			text_color);

	putText(plot,
			"iron curtain",
			Point2i(plot_size_x / 2 - 37, plot_size_y - 9),
			FONT_HERSHEY_PLAIN,
			1,
			Scalar(0, 0, 255));

  float normalization = sqrt(objectNormal.x * objectNormal.x + objectNormal.y * objectNormal.y);
  float scale = 0.5;
  float golden_x = objectPosition.x - scale * objectNormal.x / normalization;
  float golden_y = objectPosition.y - scale * objectNormal.y / normalization;
  float new_yaw = atan2(objectPosition.y - quadPosition.y, objectPosition.x - quadPosition.x) * 180 / M_PI;

  golden_x = golden_x / real_size_x * plot_size_x + plot_size_x / 2;
  golden_y = golden_y / real_size_y * plot_size_y + plot_size_y / 2;
 	cv::Point2i goldenPoint = cv::Point2i(golden_x, golden_y);
	cv::circle(plot, goldenPoint, 1, Scalar(0, 0, 255), 2);
	cv::Vector<Point3f> coordinates;
	vector<string> labels;

	coordinates.resize(7);

	cv::Vec3f distanceVector = cv::Vec3f(objectPosition) - cv::Vec3f(quadPosition);
	cv::Point3f distance = cv::Point3f(sqrt(distanceVector[0] * distanceVector[0] +
			distanceVector[1] * distanceVector[1] +
			distanceVector[2] * distanceVector[2]),
			0,
			0);

	coordinates[0] = objectPosition; labels.push_back("P.Hoop Point");
	coordinates[1] = objectNormal;	 labels.push_back("P.Hoop Normal");
	coordinates[2] = quadPosition;   labels.push_back("P.Quad Point");
	coordinates[3] = quadOrientation;labels.push_back("O.Quad Orient.");
	coordinates[4] = distance;labels.push_back("D.Distance.");
	coordinates[5] = cv::Point3f(0, 0, atan2(objectPosition.y - quadPosition.y, objectPosition.x - quadPosition.x));
			 labels.push_back("O.DestinationYaw");
	coordinates[6] = cv::Point3f(golden_x, golden_y, 0); labels.push_back("P.khaltak");

	plotCoordinates(plot, coordinates, labels);
	finalize(plot);
}

void WorldPlotter::plotCoordinates(Mat &plot, Vector<Point3f> &coordinates,
		vector<string> &labels) {
	int count = coordinates.size();
	int precision = 2;
	int width = 10;
	String x = "x", y = "y", z = "z";
	String label; 

	for(int i = 0; i < count; ++i) {
		stringstream sstr;

    label = &labels.at(i)[2];
		sstr << left << label.c_str() << right;
			  
		if(labels.at(i)[0] == 'D'){
			x = "D"; y = ""; z = "";
		} else if(labels.at(i)[0] == 'O') {
			x = "r"; y = "p"; z = "y";
		}

		putText(plot, sstr.str(), Point2i(10, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
				text_color);

		stringstream sstrx;
		sstrx.precision(precision);
		sstrx.setf(ios::fixed, ios::floatfield);

		sstrx << "> "<<x<<": ";
		sstrx.width(width);
		sstrx << right << coordinates[i].x;

		putText(plot, sstrx.str(), Point2i(120, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
				text_color);

		if(labels.at(i)[0] == 'D') continue;
		stringstream sstry;
		sstry.precision(precision);
		sstry.setf(ios::fixed, ios::floatfield);

		sstry << " "<<y<<": ";
		sstry.width(width);
		sstry << right << coordinates[i].y;

		putText(plot, sstry.str(), Point2i(280, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
				text_color);

		stringstream sstrz;
		sstrz.precision(precision);
		sstrz.setf(ios::fixed, ios::floatfield);

		sstrz << " "<<z<<": ";
		sstrz.width(width);
		sstrz << right << coordinates[i].z;

		putText(plot, sstrz.str(), Point2i(430, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
				text_color);
	}
}

void WorldPlotter::finalize(Mat& plot){

	namedWindow("TopViewPlot");
	imshow("TopViewPlot", plot);
}

void WorldPlotter::plotTrace(Mat& plot, Vector<Point2f> coordinates, Scalar color) {
	for (unsigned int i = 0; i < coordinates.size() - 1; ++i) {
		line(plot, coordinates[i], coordinates[i + 1], color);
	}
}

void WorldPlotter::plotAxes(cv::Mat& plot){
  
  int marker_x, marker_y;
  line(plot, cvPoint(plot_size_x / 2, 0),
			cvPoint(plot_size_x / 2, plot_size_y), x_color);
	line(plot, cvPoint(0, plot_size_y / 2),
			cvPoint(plot_size_x, plot_size_y / 2), y_color);

	for(int i = -real_size_x/2; i < real_size_x/2; i++){

    marker_x = i / real_size_x * plot_size_x
			+ plot_size_x / 2;
    marker_y = plot_size_y / 2;
	  line(plot, cv::Point(marker_x, marker_y - marker_size), cv::Point(marker_x, marker_y + marker_size), x_color);

    stringstream sstr; sstr<<i;
	  putText(plot, 
	  sstr.str(),
		Point2i(marker_x, marker_y - marker_size * 1.5),
  	FONT_HERSHEY_PLAIN,
		font_scale,
		x_color);
	}
	
		for(int i = -real_size_y/2; i < real_size_y/2; i++){

    marker_y = i / real_size_y * plot_size_y
			+ plot_size_y / 2;
    marker_x = plot_size_x / 2;
	  line(plot, cv::Point(marker_x, marker_y - marker_size), cv::Point(marker_x, marker_y + marker_size), x_color);

    stringstream sstr; sstr<<i;
	  putText(plot, 
	  sstr.str(),
		Point2i(marker_x + marker_size * 1.5, marker_y),
  	FONT_HERSHEY_PLAIN,
		font_scale,
		y_color);
	}
}
