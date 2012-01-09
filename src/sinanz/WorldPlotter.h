/*
 * WorldPlotter.h
 *
 *  Created on: Jan 5, 2012
 *      Author: root
 */

#ifndef WORLDPLOTTER_H_
#define WORLDPLOTTER_H_
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

class WorldPlotter {
public:
	int plot_size_x;
	int plot_size_y;
	float real_size_x;
	float real_size_y;
	int marker_size;

	Scalar x_color;
	Scalar y_color;
	Scalar normal_color;
	Scalar object_color;
	Scalar quad_color;
	Scalar text_color;
  double font_scale;

	int normal_thickness;
	int object_size;
	int object_thickness;

	Vector<Point2f> object_trace, quad_trace;

	WorldPlotter();
	virtual ~WorldPlotter();
	void plotTopView(
			Point3f objectPosition,
			Point3f objectNormal,
			Point3f quadPosition,
			Point3f quadOrientation);

	//void plotCoordinates(Vector<Point3f> coordinates);
	void plotCoordinates(Mat &plot,
			Vector<Point3f> &coordinates,
			vector<string> &labels);

	void plotTrace(Mat& plot, Vector<Point2f> coordinates, Scalar color);
	void finalize(Mat& plot);
	void plotAxes(cv::Mat& plot);
};

#endif /* WORLDPLOTTER_H_ */
