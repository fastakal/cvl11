#ifndef STEREOPROC_H
#define STEREOPROC_H

#include <opencv2/calib3d/calib3d.hpp>

class StereoProc
{
public:
	bool init(const std::string& configFile);
	bool getImageInfo(cv::Mat& intrinsicMat) const;
	bool process(cv::Mat& img1, cv::Mat& img2,
				 cv::Mat& imgRectified, cv::Mat& imgDepth);

private:
	cv::StereoBM mStereoBM;

	cv::Mat mImgRectifiedLeft;
	cv::Mat mImgRectifiedRight;
	cv::Mat mImgDisparity;

	double mBaseline;
	double mFocalLength;

	cv::Mat mIntrinsicMat;

	cv::Mat mUndistortMapXLeft;
	cv::Mat mUndistortMapYLeft;
	cv::Mat mUndistortMapXRight;
	cv::Mat mUndistortMapYRight;
};

#endif
