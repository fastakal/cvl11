#include "CodeContainer.h"
#include "SegmentedDepthImage.h"



#include <inttypes.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <time.h>
// BOOST includes
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
// OpenCV includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "StereoProc.h"

#include <interface/shared_mem/PxSHMImageServer.h>
#include "mavconn.h"

#include <deque>
#include <iostream>


using namespace std;
namespace config = boost::program_options;
namespace bfs = boost::filesystem;

// Variables
int g_minimumLengthOfAcceptedContours = 250;
int g_maximumLengthOfAcceptedContours = 500;

int g_CannyThreshold = 255;
int g_contourApproxOrder = 0;
int g_dilate = 0;
int g_line_size_for_plotting = 4;
int g_distance_between_lines = 10;
int g_angle_threshold_ratio = 5;


class pxIMUImageData {
public:
	inline pxIMUImageData(void) {}
	inline ~pxIMUImageData(void) {}

	inline pxIMUImageData(uint64_t timestamp, float roll, float pitch, float yaw, float lat, float lon, float alt, float local_z, float gx, float gy, float gz) {
		init(timestamp, roll, pitch, yaw, lat, lon, alt, local_z, gx, gy, gz);
	}

	// Initialize member variables
	inline void init(uint64_t timestamp, float roll, float pitch, float yaw, float lat, float lon, float alt, float local_z, float gx, float gy, float gz) {

		m_timestamp = timestamp;
		m_roll = roll;
		m_pitch = pitch;
		m_yaw = yaw;
		m_lat = lat;
		m_lon = lon;
		m_alt = alt;
		m_local_z = local_z;
		m_gx = gx;
		m_gy = gy;
		m_gz = gz;
	}

	uint64_t m_timestamp;
	float m_roll, m_pitch, m_yaw, m_lat, m_lon, m_alt, m_local_z, m_gx, m_gy, m_gz;
};


float colormap_jet[128][3] =
{
		{0.0f,0.0f,0.53125f},
		{0.0f,0.0f,0.5625f},
		{0.0f,0.0f,0.59375f},
		{0.0f,0.0f,0.625f},
		{0.0f,0.0f,0.65625f},
		{0.0f,0.0f,0.6875f},
		{0.0f,0.0f,0.71875f},
		{0.0f,0.0f,0.75f},
		{0.0f,0.0f,0.78125f},
		{0.0f,0.0f,0.8125f},
		{0.0f,0.0f,0.84375f},
		{0.0f,0.0f,0.875f},
		{0.0f,0.0f,0.90625f},
		{0.0f,0.0f,0.9375f},
		{0.0f,0.0f,0.96875f},
		{0.0f,0.0f,1.0f},
		{0.0f,0.03125f,1.0f},
		{0.0f,0.0625f,1.0f},
		{0.0f,0.09375f,1.0f},
		{0.0f,0.125f,1.0f},
		{0.0f,0.15625f,1.0f},
		{0.0f,0.1875f,1.0f},
		{0.0f,0.21875f,1.0f},
		{0.0f,0.25f,1.0f},
		{0.0f,0.28125f,1.0f},
		{0.0f,0.3125f,1.0f},
		{0.0f,0.34375f,1.0f},
		{0.0f,0.375f,1.0f},
		{0.0f,0.40625f,1.0f},
		{0.0f,0.4375f,1.0f},
		{0.0f,0.46875f,1.0f},
		{0.0f,0.5f,1.0f},
		{0.0f,0.53125f,1.0f},
		{0.0f,0.5625f,1.0f},
		{0.0f,0.59375f,1.0f},
		{0.0f,0.625f,1.0f},
		{0.0f,0.65625f,1.0f},
		{0.0f,0.6875f,1.0f},
		{0.0f,0.71875f,1.0f},
		{0.0f,0.75f,1.0f},
		{0.0f,0.78125f,1.0f},
		{0.0f,0.8125f,1.0f},
		{0.0f,0.84375f,1.0f},
		{0.0f,0.875f,1.0f},
		{0.0f,0.90625f,1.0f},
		{0.0f,0.9375f,1.0f},
		{0.0f,0.96875f,1.0f},
		{0.0f,1.0f,1.0f},
		{0.03125f,1.0f,0.96875f},
		{0.0625f,1.0f,0.9375f},
		{0.09375f,1.0f,0.90625f},
		{0.125f,1.0f,0.875f},
		{0.15625f,1.0f,0.84375f},
		{0.1875f,1.0f,0.8125f},
		{0.21875f,1.0f,0.78125f},
		{0.25f,1.0f,0.75f},
		{0.28125f,1.0f,0.71875f},
		{0.3125f,1.0f,0.6875f},
		{0.34375f,1.0f,0.65625f},
		{0.375f,1.0f,0.625f},
		{0.40625f,1.0f,0.59375f},
		{0.4375f,1.0f,0.5625f},
		{0.46875f,1.0f,0.53125f},
		{0.5f,1.0f,0.5f},
		{0.53125f,1.0f,0.46875f},
		{0.5625f,1.0f,0.4375f},
		{0.59375f,1.0f,0.40625f},
		{0.625f,1.0f,0.375f},
		{0.65625f,1.0f,0.34375f},
		{0.6875f,1.0f,0.3125f},
		{0.71875f,1.0f,0.28125f},
		{0.75f,1.0f,0.25f},
		{0.78125f,1.0f,0.21875f},
		{0.8125f,1.0f,0.1875f},
		{0.84375f,1.0f,0.15625f},
		{0.875f,1.0f,0.125f},
		{0.90625f,1.0f,0.09375f},
		{0.9375f,1.0f,0.0625f},
		{0.96875f,1.0f,0.03125f},
		{1.0f,1.0f,0.0f},
		{1.0f,0.96875f,0.0f},
		{1.0f,0.9375f,0.0f},
		{1.0f,0.90625f,0.0f},
		{1.0f,0.875f,0.0f},
		{1.0f,0.84375f,0.0f},
		{1.0f,0.8125f,0.0f},
		{1.0f,0.78125f,0.0f},
		{1.0f,0.75f,0.0f},
		{1.0f,0.71875f,0.0f},
		{1.0f,0.6875f,0.0f},
		{1.0f,0.65625f,0.0f},
		{1.0f,0.625f,0.0f},
		{1.0f,0.59375f,0.0f},
		{1.0f,0.5625f,0.0f},
		{1.0f,0.53125f,0.0f},
		{1.0f,0.5f,0.0f},
		{1.0f,0.46875f,0.0f},
		{1.0f,0.4375f,0.0f},
		{1.0f,0.40625f,0.0f},
		{1.0f,0.375f,0.0f},
		{1.0f,0.34375f,0.0f},
		{1.0f,0.3125f,0.0f},
		{1.0f,0.28125f,0.0f},
		{1.0f,0.25f,0.0f},
		{1.0f,0.21875f,0.0f},
		{1.0f,0.1875f,0.0f},
		{1.0f,0.15625f,0.0f},
		{1.0f,0.125f,0.0f},
		{1.0f,0.09375f,0.0f},
		{1.0f,0.0625f,0.0f},
		{1.0f,0.03125f,0.0f},
		{1.0f,0.0f,0.0f},
		{0.96875f,0.0f,0.0f},
		{0.9375f,0.0f,0.0f},
		{0.90625f,0.0f,0.0f},
		{0.875f,0.0f,0.0f},
		{0.84375f,0.0f,0.0f},
		{0.8125f,0.0f,0.0f},
		{0.78125f,0.0f,0.0f},
		{0.75f,0.0f,0.0f},
		{0.71875f,0.0f,0.0f},
		{0.6875f,0.0f,0.0f},
		{0.65625f,0.0f,0.0f},
		{0.625f,0.0f,0.0f},
		{0.59375f,0.0f,0.0f},
		{0.5625f,0.0f,0.0f},
		{0.53125f,0.0f,0.0f},
		{0.5f,0.0f,0.0f}
};

void
colorDepthImage(cv::Mat& imgDepth, cv::Mat& imgColoredDepth)
{
	imgColoredDepth = cv::Mat::zeros(imgDepth.size(), CV_8UC3);

	for (int i = 0; i < imgColoredDepth.rows; ++i)
	{
		const float* depth = imgDepth.ptr<float>(i);
		unsigned char* pixel = imgColoredDepth.ptr<unsigned char>(i);
		for (int j = 0; j < imgColoredDepth.cols; ++j)
		{
			if (depth[j] != 0)
			{
				int idx = fmin(depth[j], 10.0f) / 10.0f * 127.0f;
				idx = 127 - idx;

				pixel[0] = colormap_jet[idx][2] * 255.0f;
				pixel[1] = colormap_jet[idx][1] * 255.0f;
				pixel[2] = colormap_jet[idx][0] * 255.0f;
			}

			pixel += 3;
		}
	}
}

/**
 * A function that creates a window with trackbars controlling parameters to test...
 */
void createControlPanel(){
	namedWindow("Control Panel");
	createTrackbar("Threshold", "Control Panel", &g_CannyThreshold, 255, 0);
	createTrackbar("MinLengthContours", "Control Panel", &g_minimumLengthOfAcceptedContours, 500, 0);
	createTrackbar("MaxLengthContours", "Control Panel", &g_maximumLengthOfAcceptedContours, 500, 0);
	createTrackbar("contourApproxOrder", "Control Panel", &g_contourApproxOrder, 10, 0);
	createTrackbar("dilate", "Control Panel", &g_dilate, 1, 0);
	createTrackbar("Line-Size for plotting", "Control Panel", &g_line_size_for_plotting, 100, 0);
	createTrackbar("Distance between lines", "Control Panel", &g_distance_between_lines, 100, 0);
	createTrackbar("Angle Threshold", "Control Panel", &g_angle_threshold_ratio, 10, 0);
}

////////////////////////////////////////////////////////////////MAIN
////////////////////////////////////////////////////////////////MAIN
////////////////////////////////////////////////////////////////MAIN
////////////////////////////////////////////////////////////////MAIN
////////////////////////////////////////////////////////////////MAIN
////////////////////////////////////////////////////////////////MAIN
int main(int argc, char* argv[])
{

	std::string strImageDataFile;
	std::string strVoctree;
	std::vector<pxIMUImageData> vecImageData;
	std::string imagepath_left;
	std::string imagepath_right;
	std::string stereo_calib;
	std::string cachedir;
	uint64_t camid_left = 0;  ///< Left Camera unique id
	uint64_t camid_right = 0; ///< Right camera unique id
	bool silent, verbose;
	int sysid = getSystemID();
	int compid = PX_COMP_ID_CAMERA;

	// ----- Handling Program options
	config::options_description desc("Allowed options");
	desc.add_options()
										("help", "produce help message")
										("imagedata,i", config::value<std::string>(&strImageDataFile)->default_value(""), "full path of imagedata.txt file to be loaded")
										("images_left", config::value<std::string>(&imagepath_left)->default_value(""), "Path to images of the left camera")
										("images_right", config::value<std::string>(&imagepath_right)->default_value(""), "Path to images of the right camera (if not defined: single camera)")
										("stereo_calib,c", config::value<std::string>(&stereo_calib)->default_value(""), "Path to stereo calibration file")
										("camid_left", config::value<uint64_t>(&camid_left), "Left camera ID to be simulated")
										("camid_right", config::value<uint64_t>(&camid_right), "Right camera ID to be simulated")
										("silent,s", config::bool_switch(&silent)->default_value(false), "suppress outputs")
										("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
										;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	uint64_t timestamp;
	float roll, pitch, yaw, lat, lon, alt, local_z, gx, gy, gz;

	// Open log data file
	// Load the calibration file
	std::ifstream infile(strImageDataFile.c_str(), std::ios::in);
	if (!infile.is_open())
	{
		printf("Unable to open image data file\n");
		return 0;	// terminate with error
	}
	else
	{
		printf("Opened image data file\n");
	}

	//try standard folders for left and right images if they were not defined
	if(imagepath_left.length() == 0)
	{
		imagepath_left = strImageDataFile.substr(0, strImageDataFile.length()-13) + "left";
		printf("Left images stream path not specified, trying %s\n", imagepath_left.c_str());
	}

	if(imagepath_right.length() == 0)
	{
		imagepath_right = strImageDataFile.substr(0, strImageDataFile.length()-13) + "right";
		printf("Right images stream path not specified, trying %s\n", imagepath_right.c_str());
	}

	StereoProc imgproc;
	cv::Mat intrinsicMat;
	if (!imgproc.init(stereo_calib))
	{
		if (stereo_calib.length() == 0)
		{
			printf("NO STEREO Calibration given - specify file name with --stereo_calib.\n");
		}
		else
		{
			printf("FAILED to find stereo config file at %s\n", stereo_calib.c_str());
		}
		exit(EXIT_FAILURE);
	}
	imgproc.getImageInfo(intrinsicMat);

	std::string line;
	std::string element;
	uint64_t lineCount = 0;
	uint64_t oldtimestamp=0;
	size_t numImages = 10000;

	while (std::getline(infile, line) && vecImageData.size() <= numImages)
	{
		std::istringstream csvStream(line);
		lineCount++;

		if (line.size() == 0 || line.at(0) == '#' || lineCount < 7) // skip header
		{
			// Ignore this line
			continue;
		}
		else
		{
			// Get timestamp
			std::getline(csvStream, element, ',');
			std::stringstream ssStream(element);
			ssStream >> timestamp;
			// ROLL
			std::getline(csvStream, element, ',');
			std::stringstream ssStream2(element);
			ssStream2 >> roll;
			// PITCH
			std::getline(csvStream, element, ',');
			std::stringstream ssStream3(element);
			ssStream3 >> pitch;
			// YAW
			std::getline(csvStream, element, ',');
			std::stringstream ssStream4(element);
			ssStream4 >> yaw;
			// LAT
			std::getline(csvStream, element, ',');
			std::stringstream ssStream5(element);
			ssStream5 >> lat;
			// LON
			std::getline(csvStream, element, ',');
			std::stringstream ssStream6(element);
			ssStream6 >> lon;
			// ALT
			std::getline(csvStream, element, ',');
			std::stringstream ssStream7(element);
			ssStream7 >> alt;
			// LOCAL_Z
			std::getline(csvStream, element, ',');
			std::stringstream ssStream8(element);
			ssStream8 >> local_z;
			// GX
			std::getline(csvStream, element, ',');
			std::stringstream ssStream9(element);
			ssStream9 >> gx;
			// GY
			std::getline(csvStream, element, ',');
			std::stringstream ssStream10(element);
			ssStream10 >> gy;
			// GY
			std::getline(csvStream, element, ',');
			std::stringstream ssStream11(element);
			ssStream11 >> gz;

			if (timestamp > oldtimestamp /*&& lineCount > 100*/) {  // skip older entries that might occur
				//std::cout << timestamp << ", " << roll << ", " << pitch << ", " << yaw << ", " << lat << ", " << lon << ", " << alt << ", " << local_z << ", " << gx << ", " << gy << ", " << gz << std::endl;
				vecImageData.push_back(pxIMUImageData(timestamp, roll, pitch, yaw, lat, lon, alt, local_z, gx, gy, gz));
				oldtimestamp = timestamp;
			}
		}
	}

	char strImgLeft[1024];
	char strImgRight[1024];
	char strout[1024];

	cv::Mat imgLeft, imgRight, imgDatabase, imgDepth, imgRectified;
	//process image list
	for (size_t i = 0; i < vecImageData.size(); i++)
	{
		printf("# INFO: Processing Image: %zu...\n", i);

		sprintf(strImgLeft, "%s/%016llu.bmp", imagepath_left.c_str(), vecImageData[i].m_timestamp);
		sprintf(strImgRight, "%s/%016llu.bmp", imagepath_right.c_str(), vecImageData[i].m_timestamp);
		imgLeft = cv::imread(strImgLeft,0);  // image forced to be grayscale
		imgRight = cv::imread(strImgRight,0);  // image forced to be grayscale

		if (!imgLeft.data || !imgRight.data)
		{
			printf("fail.\n");
			perror("Image not found. Exiting.\n");
			continue;
		}


		// Compute Stereo and store the processed frames in buffer
		imgproc.process(imgLeft, imgRight, imgRectified, imgDepth);
		cv::Mat* imgDepthCopy = new cv::Mat();
		imgDepth.copyTo(*imgDepthCopy);

		cv::Mat imgDepthColor;

		colorDepthImage(imgDepth, imgDepthColor);

		//std::cout << strImgLeft << " x: " << vecImageData[i].m_lat << " y: " << vecImageData[i].m_lon << " z: " << vecImageData[i].m_alt << std::endl;  // OPTICAL FLOW DATA

		cv::Mat img1 = cv::Mat::zeros(imgLeft.size(), CV_8UC3);
		cv::Mat img2 = cv::Mat::zeros(imgRight.size(), CV_8UC3);
		cv::Mat img3 = cv::Mat::zeros(imgDepth.size(), CV_8UC3);
		cv::Mat output_image;
		imgLeft.copyTo(img1);
		imgRight.copyTo(img2);

		std::cout<<"stereo_calib: "<<stereo_calib<<"\n";
		int newSizeX = imgLeft.cols/2;
		int newSizeY = imgLeft.rows/2;
		int marginBetweenWindows = 20;

		cv::namedWindow("Left Image (Forward Camera)",0);
		cv::imshow("Left Image (Forward Camera)", imgLeft);
		cvResizeWindow("Left Image (Forward Camera)", newSizeX, newSizeY);
		cvMoveWindow("Left Image (Forward Camera)", marginBetweenWindows, 0);

		cv::namedWindow("Right Image (Forward Camera)",0);
		cv::imshow("Right Image (Forward Camera)", imgRight);
		cvResizeWindow("Right Image (Forward Camera)", newSizeX, newSizeY);
		cvMoveWindow("Right Image (Forward Camera)", newSizeX + 2*marginBetweenWindows, 0);

		cv::namedWindow("Depth map",0);
		cv::imshow("Depth map", imgDepthColor);
		cvResizeWindow("Depth map", newSizeX, newSizeY);
		cvMoveWindow("Depth map", 2*(newSizeX + 2*marginBetweenWindows), 0);


		cv::waitKey(1);



		createControlPanel();
		double angle_treshold = ((double)g_angle_threshold_ratio)/10;

		imgDepthColor.copyTo(img3);

		CodeContainer myCode1 = CodeContainer(imgRectified, img3, imgDepth,
				g_minimumLengthOfAcceptedContours,
				g_maximumLengthOfAcceptedContours,
				g_CannyThreshold,
				g_contourApproxOrder,
				g_dilate,
				g_line_size_for_plotting,
				g_distance_between_lines,
				false,
				angle_treshold
		);
	}

	return EXIT_SUCCESS;
}



