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

	real_size_x = 6;
	real_size_y = 6;
	x_color = Scalar(255/2, 255/2, 255/2);
	y_color = Scalar(255/2, 255/2, 255/2);
	normal_color = Scalar(0, 0, 255);
	normal_thickness = 2;
	object_size = 4;
	object_color = Scalar(128, 128, 0);
	object_thickness = 3;
	quad_color = Scalar(0, 128, 128);
	text_color = Scalar(255, 255, 255);
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

	line(plot, cvPoint(plot_size_x / 2, 0),
			cvPoint(plot_size_x / 2, plot_size_y), x_color);
	line(plot, cvPoint(0, plot_size_y / 2),
			cvPoint(plot_size_x, plot_size_y / 2), y_color);

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

	cv::Vector<Point3f> coordinates;
	vector<string> labels;

	coordinates.resize(4);

	coordinates[0] = objectPosition; labels.push_back("Hoop Point");
	coordinates[1] = objectNormal;	 labels.push_back("Hoop Normal");
	coordinates[2] = quadPosition;   labels.push_back("Quad Point");
	coordinates[3] = quadOrientation;labels.push_back("Quad Orient.");


	plotCoordinates(plot, coordinates, labels);
	finalize(plot);
}
/*
void WorldPlotter::plotCoordinates(Vector<Point3f> coordinates){

	std::stringstream sstr1;
	std::string str1;
	sstr1 << "Hoop Point:"<<
			" x: "<<coordinates[0].x<<
			".y: "<<coordinates[0].y<<
			".z: "<<coordinates[0].z<<
			"\n";
	str1 = sstr1.str();

	putText(plot, str1, Point2i(100, 20), FONT_HERSHEY_PLAIN, 1,text_color);

	std::stringstream sstr2;
	sstr2 << "Hoop Normal:"<<
			" x: "<<coordinates[1].x<<
			".y: "<<coordinates[1].y<<
			".z: "<<coordinates[1].z<<
			"\n";
	str1 = sstr2.str();
	putText(plot, str1, Point2i(100, 40), FONT_HERSHEY_PLAIN, 1,text_color);

	std::stringstream sstr3;
	sstr3 << "Quad Point:"<<
			" x: "<<coordinates[2].x<<
			".y: "<<coordinates[2].y<<
			".z: "<<coordinates[2].z<<
			"\n";
	str1 = sstr3.str();
	putText(plot, str1, Point2i(100, 60), FONT_HERSHEY_PLAIN, 1,text_color);

	std::stringstream sstr4;
	sstr4 << "Quad Orientation:"<<
			" x: "<<coordinates[3].x<<
			".y: "<<coordinates[3].y<<
			".z: "<<coordinates[3].z<<
			"\n";
	str1 = sstr4.str();
	putText(plot, str1, Point2i(100, 80), FONT_HERSHEY_PLAIN, 1,text_color);
}
 */
void WorldPlotter::plotCoordinates(Mat &plot, Vector<Point3f> &coordinates,
		vector<string> &labels) {
	int count = coordinates.size();

	for(int i = 0; i < count; ++i) {
		stringstream sstr;

		sstr << left << labels.at(i).c_str() << right;

		putText(plot, sstr.str(), Point2i(10, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
				text_color);

		stringstream sstrx;
		sstrx.precision(5);
		sstrx.setf(ios::fixed, ios::floatfield);

		sstrx << "> x: ";
		sstrx.width(10);
		sstrx << right << coordinates[i].x;

		putText(plot, sstrx.str(), Point2i(120, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
				text_color);

		stringstream sstry;
		sstry.precision(5);
		sstry.setf(ios::fixed, ios::floatfield);

		sstry << " y: ";
		sstry.width(10);
		sstry << right << coordinates[i].y;

		putText(plot, sstry.str(), Point2i(280, 15 * (i + 1)), FONT_HERSHEY_PLAIN, 1,
				text_color);

		stringstream sstrz;
		sstrz.precision(5);
		sstrz.setf(ios::fixed, ios::floatfield);

		sstrz << " z: ";
		sstrz.width(10);
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
