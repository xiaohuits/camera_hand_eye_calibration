#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>

void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 5, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(500, 500), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
    ROS_INFO("image saved 'BoardImage.jpg' ");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_charuco_board");
  createBoard();
  ros::spin();
  return 0;
}