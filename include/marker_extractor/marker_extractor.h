#ifndef MARKER_EXTRACTOR_H
#define MARKER_EXTRACTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>

class MarkerExtractor
{
public:
    MarkerExtractor();
    ~MarkerExtractor();
private:

    ros::NodeHandle n_;
    image_transport::Subscriber img_sub_;
    ros::Subscriber info_sub_;
    image_transport::Publisher img_pub_;
    ros::Publisher pose_pub_;
    sensor_msgs::CameraInfo cam_info_;

    // parameters
    int p_width_;
    int p_height_;
    int p_scale_;
    bool doRansac_;
    int iterationsCount_;
    double reprojectionError_;
    int minInliersCount_;

    std::vector<cv::Point3f> pts3d_;
    cv::Mat K_;
    cv::Mat d_;
    bool calibration_set_;

    void parseCalibration();
    void cbImage(const sensor_msgs::ImageConstPtr &msg);
    void cbCameraInfo(const sensor_msgs::CameraInfoConstPtr & msg);

};

#endif // MARKER_EXTRACTOR_H
