/*============================================================
    Created by Hui Xiao - University of Washington - 2020 
    huix27@uw.edu
==============================================================*/

#include <iostream>
#include <vector>
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include <math.h>

using namespace std;
using namespace cv;

class Calibrator 
{
public:
    Calibrator() : it(nh)
    {
        img_sub = it.subscribe("avt_camera_img", 2, &Calibrator::imageCb, this);
        setDetectionParameters();
    }

    // set detection parameters
    void setDetectionParameters();

    // marker detection function
    // return 1 if has marker detected
    int detectMarkers(Mat& image);

    int showImage();

    void saveData(){
        save_frame = true;
    }

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::CharucoBoard> board; 
    
    Mat processed_img = Mat::zeros(1200, 1600, CV_8UC3);

    std::vector<std::vector<cv::Point2f>> allCharucoCorners;
    std::vector<std::vector<int>> allCharucoIds;

    bool save_frame = false;
    // image call back function
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    // help function: convert rotation and translation vectors to a ROS pose message type
    geometry_msgs::Pose rtv2pose(Vec3d rvec, Vec3d tvec);
};

void Calibrator::setDetectionParameters()
{
    ros::NodeHandle pnh("~");
    double _minMarkerPerimeterRate, _maxMarkerPerimeterRate;
    double _squareLength, _markerLength;
    if(pnh.getParam("minMarkerPerimeterRate", _minMarkerPerimeterRate))
    {
        detectorParams->minMarkerPerimeterRate = _minMarkerPerimeterRate;
    }
    else
    {
        //otherwist a dafault value is used.
        ROS_ERROR("can't load parameter 'minMarkerPerimeterRate' ");
    }
    if(pnh.getParam("maxMarkerPerimeterRate", _maxMarkerPerimeterRate))
    {
        detectorParams->maxMarkerPerimeterRate = _maxMarkerPerimeterRate;
    }
    else
    {
        //otherwist a dafault value is used.
        ROS_ERROR("can't load parameter 'maxMarkerPerimeterRate' ");
    }
    if(pnh.getParam("square_length", _squareLength))
    {
    }
    else
    {
        //otherwist a dafault value is used.
        ROS_ERROR("can't load parameter 'marker_length' ");
        _squareLength = 0.02244;
    }
    if(pnh.getParam("marker_length", _markerLength))
    {
    }
    else
    {
        //otherwist a dafault value is used.
        ROS_ERROR("can't load parameter 'marker_separation' ");
        _markerLength = 0.01122;
    }

    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    //detectorParams->cornerRefinementWinSize = 3;
    //detectorParams->cornerRefinementMaxIterations = 8;
    //detectorParams->perspectiveRemoveIgnoredMarginPerCell = 0.3;
    //detectorParams->errorCorrectionRate = 0.8;
    board = aruco::CharucoBoard::create(8, 10, _squareLength, _markerLength, dictionary);
}

void Calibrator::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // Note on timming:
    // Seems that marker detection and imgage pub is time comsuming.
    // Converting OpenCV and ROS images is less time-comsuming.

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(processed_img);

    // detect ChArucoBoard
    vector<int> markerIds;
    vector<vector<Point2f> > markerCorners;
    aruco::detectMarkers(cv_ptr->image, board->dictionary, markerCorners, markerIds, detectorParams);
    if (markerIds.size() > 0)
    {
        aruco::drawDetectedMarkers(processed_img, markerCorners, markerIds);
        vector<cv::Point2f> charucoCorners;
        vector<int> charucoIds;
        aruco::interpolateCornersCharuco(markerCorners, markerIds, cv_ptr->image, board, charucoCorners, charucoIds);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0)
            cv::aruco::drawDetectedCornersCharuco(processed_img, charucoCorners, charucoIds, cv::Scalar(0, 0, 255));
            if (save_frame)
            {
                ROS_INFO("adding one frame....");
                allCharucoCorners.push_back(charucoCorners);
                allCharucoIds.push_back(charucoIds);
                ROS_INFO("%d frames collected", (int)allCharucoCorners.size());
                save_frame = false;
            }
    }
}

int Calibrator::showImage()
{
    namedWindow("image", WINDOW_NORMAL);
    imshow("image", processed_img);
    return waitKey(1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_hand_eye_calibration");
  Calibrator obj;
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
      int key = obj.showImage();
      if(key == 's')
      {
          obj.saveData();
      }
      if(key == 'q')
      {
          
      }
      //ROS_INFO("key pressed %d", key);
      ros::spinOnce();
  }
  return 0;
}