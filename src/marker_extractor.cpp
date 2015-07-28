#include "marker_extractor/marker_extractor.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PoseStamped.h"
#include "icg_msgs/icg_get_calibration.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

using namespace std;

MarkerExtractor::MarkerExtractor():
    calibration_set_(false)
{
    // parse parameters
    ros::NodeHandle pNode("~");
    pNode.param("pattern_width", p_width_, 8);
    pNode.param("pattern_height", p_height_, 5);
    pNode.param("pattern_scale", p_scale_, 3);
    pNode.param("robust", doRansac_, true);
    if(doRansac_)
    {
        pNode.param("robust_iteration_count", iterationsCount_, 100);
        pNode.param("robust_max_reprojection_error", reprojectionError_, 0.1);
        pNode.param("robust_min_inliers_count", minInliersCount_, 30);
    }


    image_transport::ImageTransport it(pNode);
    img_sub_ = it.subscribe("image_rect", 1, &MarkerExtractor::cbImage, this);
    img_pub_ = it.advertise("output", 1);
    pose_pub_ = pNode.advertise<geometry_msgs::PoseStamped>("pose",1);

    // create 3d points
    pts3d_.clear();
    for (int c = 0; c < p_width_; c++)
        for(int r = 0; r < p_height_; r++)
            pts3d_.push_back( cv::Point3d( c, r, 0.0) * p_scale_ );

    // camera_info from serice call
    ros::ServiceClient client = pNode.serviceClient<icg_msgs::icg_get_calibration>("get_calibration");
    icg_msgs::icg_get_calibration srv;
    srv.request.request = true;
    if(client.call(srv))
    {
        ROS_INFO_STREAM("Got calibration from serviece" << client.getService());
        cam_info_.height = srv.response.height;
        cam_info_.width = srv.response.width;
        cam_info_.K[0] = srv.response.fx * cam_info_.width; //fx
        cam_info_.K[1] = 0.0;
        cam_info_.K[2] = srv.response.cx * cam_info_.width; //cx
        cam_info_.K[3] = 0.0;
        cam_info_.K[4] = srv.response.fy * cam_info_.height; //fy
        cam_info_.K[5] = srv.response.cy * cam_info_.height; //cy
        cam_info_.K[6] = 0.0;
        cam_info_.K[7] = 0.0;
        cam_info_.K[8] = 1.0;
        parseCalibration();
    }
    else
    {
        ROS_WARN("Failed to get calibration using service");
        ROS_WARN("Subscribing to camera info");
        client.shutdown();
        info_sub_ = n_.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, &MarkerExtractor::cbCameraInfo, this);
    }
}

MarkerExtractor::~MarkerExtractor()
{
}

void MarkerExtractor::parseCalibration()
{
    // assuming undistorted images
    ROS_WARN("Assuming undistorted images!");
    cam_info_.D.clear();
    for (int i = 0; i < 4; i++)
        cam_info_.D.push_back(0.0);

    // K and d as Mat
    K_ = cv::Mat (3,3,CV_64F);
    for (int i = 0; i < cam_info_.K.size(); i++)
        K_.at<double>(i) = cam_info_.K[i];

    d_ = cv::Mat (4,1,CV_64F);
    for (int i = 0; i < cam_info_.D.size(); i++)
        d_.at<double>(i) = cam_info_.D[i];

    ROS_INFO_STREAM("K: " << cam_info_.K[0] << " " << cam_info_.K[1] << " " << cam_info_.K[2]);
    ROS_INFO_STREAM("   " << cam_info_.K[3] << " " << cam_info_.K[4] << " " << cam_info_.K[5]);
    ROS_INFO_STREAM("   " << cam_info_.K[6] << " " << cam_info_.K[7] << " " << cam_info_.K[8]);

    calibration_set_ = true;
}

void MarkerExtractor::cbCameraInfo(const sensor_msgs::CameraInfoConstPtr & msg)
{
    cam_info_ = *msg;
    parseCalibration();
    info_sub_.shutdown();
}

void MarkerExtractor::cbImage(const sensor_msgs::ImageConstPtr &msg)
{
    if(!calibration_set_)
        return;

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

    cv::Mat img_bgr = cv_ptr->image;
    cv::Mat img_gray;
    cv::cvtColor(img_bgr,img_gray,cv::COLOR_BGR2GRAY);

    // find checker board
    std::vector<cv::Point2f> pts2d;

    bool found = cv::findChessboardCorners( img_gray, cvSize(p_height_,p_width_), pts2d, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (!found)
    {
        ROS_WARN("No chess board found!");
        return;
    }

    if (img_pub_.getNumSubscribers())
    {
        cv::drawChessboardCorners(img_bgr,cvSize(p_height_,p_width_), cv::Mat(pts2d), found );
        img_pub_.publish(cv_ptr->toImageMsg());
    }

    // calculate pose
    try
    {
        cv::Mat r(3,1,CV_64F);
        cv::Mat t(3,1,CV_64F);

        if(!doRansac_)
            cv::solvePnP(pts3d_,pts2d,K_,d_,r,t);
        else
        {
            cv::Mat inliers;
            cv::solvePnPRansac( pts3d_, pts2d, K_, d_, r, t, false,
                                iterationsCount_,
                                reprojectionError_,
                                minInliersCount_,
                                inliers);
            ROS_INFO_STREAM("inliers: " << inliers.rows << "x" << inliers.cols);
        }

        ROS_INFO_STREAM("r: " << r);
        ROS_INFO_STREAM("t: " << t);

        if(pose_pub_.getNumSubscribers())
        {
            tf::Quaternion q;
            q.setRPY(r.at<double>(0),r.at<double>(1),r.at<double>(2));
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose.position.x = t.at<double>(0);
            pose.pose.position.y = t.at<double>(1);
            pose.pose.position.z = t.at<double>(2);
            pose.pose.orientation.w = q.getW();
            pose.pose.orientation.x = q.getX();
            pose.pose.orientation.y = q.getY();
            pose.pose.orientation.z = q.getZ();

            cout << "Publishing" << endl;
            pose_pub_.publish(pose);
        }
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("cv exception: %s", e.what());
    }

}
