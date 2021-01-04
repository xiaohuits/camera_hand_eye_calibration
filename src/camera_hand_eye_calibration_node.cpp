/*============================================================
    Created by Hui Xiao - University of Washington - 2020 
    huix27@uw.edu
==============================================================*/

#include <iostream>
#include <vector>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
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
using geometry_msgs::TransformStamped;

class Calibrator 
{
public:
    Calibrator() : it(nh), tfListener(tfBuffer)
    {
        img_sub = it.subscribe("avt_camera_img", 2, &Calibrator::imageCb, this);
        setDetectionParameters();
    }

    // set detection parameters
    void setDetectionParameters();

    // display the processed image and return pressed key
    int showImage();

    // save the current frame
    void saveData(){
        save_frame = true;
    }

    // calibrate based on current collected frames
    void calibrate();

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::CharucoBoard> board; 
    
    //Mat processed_img;
    Mat info_activate, info_inactivate, display_img;

    vector<vector<Point2f>> allCharucoCorners;
    vector<vector<int>> allCharucoIds;
    vector<TransformStamped> allRobotPoses;
    Size imgSize;

    // flag to indicate if have received a image
    bool image_received = false;

    // flag to save current frame
    bool save_frame = false;

    // tf listener to get robot pose
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // image callback function
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    // help function: convert transformation ROS message to rotation and translation vectors 
    void transform2rv(TransformStamped transform, Mat& rvec, Mat& tvec);

    // help function: convert quaternion to angle axis
    void quat2angaxis(double x, double y, double z, double w, Mat& angaxis);

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
    
    if(!image_received)
    {
        // some initial work when first received an image
        image_received = true;
        imgSize = cv_ptr->image.size();
        int h = (int)(imgSize.height/5);
        int w = (int)imgSize.width;
        Rect ROI(0,0,w,h);
        Mat temp = cv_ptr->image(ROI);
        temp.copyTo(info_activate);
        info_activate = Scalar(255,255,255);
        info_activate.copyTo(info_inactivate);
        String info = "press 's' to add current keyframe, 'c' to calibrate, 'q' to quit program";
        String info2 = "press 'q' to quit program";
        putText(info_activate, info, Point(10,(int)(h/2)), FONT_HERSHEY_SIMPLEX, 0.9, Scalar(0, 0, 0), 4);
        putText(info_inactivate, info2, Point(10,(int)(h/2)), FONT_HERSHEY_SIMPLEX, 0.9, Scalar(0, 0, 0), 4);
    }
    Mat processed_img;
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
                //ROS_INFO("adding one frame....");
                allCharucoCorners.push_back(charucoCorners);
                allCharucoIds.push_back(charucoIds);
                // get robot pose
                TransformStamped robot_pose = tfBuffer.lookupTransform("base", "tool0_controller", ros::Time(0));
                allRobotPoses.push_back(robot_pose);
                ROS_INFO("%d frames collected", (int)allCharucoCorners.size());
                save_frame = false;
            }
        vconcat(info_activate, processed_img, display_img);
    }
    else
    {
        vconcat(info_inactivate, processed_img, display_img);
    }
    
}

int Calibrator::showImage()
{
    if(image_received)
    {
        namedWindow("image", WINDOW_NORMAL);
        imshow("image", display_img);
        return waitKey(1);
    }
}

void Calibrator::quat2angaxis(double x, double y, double z, double w, Mat& angaxis)
{
    // normorlize quaternion
    double norm = sqrt(x*x + y*y + z*z + w*w);
    x = x/norm;
    y = y/norm;
    z = z/norm;
    w = w/norm;
    // convert to angle axis
    double angle = 2*acos(w);
    double s = sqrt(1-w*w);
    double rx, ry, rz;
    if(s < 0.00001)
    {
        // angle is small, so rotation direction is not important
        rx = 1;
        ry = 0;
        rz = 0;
    }
    else
    {
        rx = x / s;
        ry = y / s;
        rz = z / s;
    }
    // normalize rotation direction
    norm = sqrt(rx*rx + ry*ry + rz*rz);
    rx = rx / norm * angle;
    ry = ry / norm * angle;
    rz = rz / norm * angle;
    angaxis.at<double>(0,0) = rx;
    angaxis.at<double>(1,0) = ry;
    angaxis.at<double>(2,0) = rz;
}

void Calibrator::transform2rv(TransformStamped transform, Mat& rvec, Mat& tvec)
{
    tvec = Mat::zeros(3,1,CV_64F);
    rvec = Mat::zeros(3,1,CV_64F);
    auto rot = transform.transform.rotation;
    auto tran = transform.transform.translation;
    quat2angaxis(rot.x, rot.y, rot.z, rot.w, rvec);
    tvec.at<double>(0,0) = tran.x;
    tvec.at<double>(1,0) = tran.y;
    tvec.at<double>(2,0) = tran.z;
    return;
}

void Calibrator::calibrate()
{
    ROS_INFO("Calibrating the camera.... %d poses collected.", (int)allCharucoCorners.size());
    
    // calibrate camera
    Mat camera_matrix, dist_coeffs;
    vector<Mat> rvecs, tvecs;
    double repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, camera_matrix, dist_coeffs, rvecs, tvecs);

    // calibrate hand eye position
    vector<Mat> R_gripper2base, t_gripper2base;
    for(int i=0; i<allRobotPoses.size(); i++)
    {
        Mat R, t;
        transform2rv(allRobotPoses[i], R, t);
        R_gripper2base.push_back(R);
        t_gripper2base.push_back(t);
    }
    Mat R_cam2gripper, t_cam2gripper;
    calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, R_cam2gripper, t_cam2gripper);
    cout << "camera matrix:\n" << camera_matrix << endl;
    cout << "distortion coefficients:\n" << dist_coeffs << endl;
    cout << "hand eye pose: translation:\n" << t_cam2gripper << endl;
    cout << "hand eye pose: rotation:\n" << R_cam2gripper << endl;

    // saving calibration results
    ofstream myfile;
    myfile.open("camera_hand_eye_calibration.yaml");
    myfile << "# This is autogenerated file to store camera and hand-eye calibration results\n";
    myfile << "camera_calibration:\n";
    myfile << "  # camera matrix\n";
    myfile << "  K: " << camera_matrix.reshape(1,1) << endl;
    myfile << "  # distortion parameters\n";
    myfile << "  D: " << dist_coeffs.reshape(1,1) << endl;
    myfile << "hand_eye_position:\n";
    myfile << "  # rotaion matrix\n";
    myfile << "  R: " << R_cam2gripper.reshape(1,1) << endl;
    myfile << "  # translation\n";
    myfile << "  t: " << t_cam2gripper.reshape(1,1) << endl;
    ROS_INFO("calibration saved to 'camera_hand_eye_calibration.yaml");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_hand_eye_calibration");
    Calibrator obj;
    ros::Rate loop_rate(50);
    cout << "press 's' to add current keyframe, 'c' to calibrate, 'q' to quit program" << endl;
    while (ros::ok())
    {
        int key = obj.showImage();
        if(key == 's')
        {
            obj.saveData();
        }
        if(key == 'c')
        {
            obj.calibrate();
        }
        if(key == 'q')
        {
            break;
        }
        ros::spinOnce();
    }
    return 0;
}