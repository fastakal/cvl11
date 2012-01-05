INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
)

SET_SOURCE_FILES(EXAMPLE_SRC_FILES
  loadimages.cc
  StereoProc.cc
)

PIXHAWK_EXECUTABLE(Main Main.cc 
WorldPlotter WorldPlotter.cpp 
StereoProc StereoProc.cc
CodeContainer CodeContainer.cpp
segmentedImage segmentedImage.cc
ContoursAndLines ContoursAndLines.cpp
LinkedLines LinkedLines.cpp
MyEllipses MyEllipses.cpp
HoopPosition HoopPosition.cpp
destination3dPoint destination3dPoint.cpp)
PIXHAWK_LINK_LIBRARIES(Main
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
  ${OPENCV_IMGPROC_LIBRARY_OPTIMIZED}
  ${OPENCV_HIGHGUI_LIBRARY_OPTIMIZED}
  
  ${OPENCV_CALIB3D_LIBRARY_OPTIMIZED}
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
  ${OPENCV_FLANN_LIBRARY}
  ${OPENCV_FEATURES2D_LIBRARY}
  
  mavconn_lcm
  mavconn_shm
  lcm
)
