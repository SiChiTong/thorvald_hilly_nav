#include <ros/ros.h>
#include <glob.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <geometry_msgs/Twist.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <Eigen/Geometry>

class HillyNav{

public:

    double rho = -0.331613;
    double ty = 0;
    double tz = 1.05;
    int hori_strips = 10;

    std::vector<cv::Point> line_fit;
    cv::Point2f P;
    cv::Point2f Q;
    Eigen::Vector3f F;
    Eigen::Vector3f F_des;
    int controller_ID = 1;
    double v = 0.05;
    geometry_msgs::Twist VelocityMsg;

    cv::Mat pred_img;
    bool pred_img_received = false;

    // ROS node handle.
    ros::NodeHandle nodeHandle_;

    ros::Subscriber pred_img_sub;
    ros::Publisher cmd_velocities;

     // HillyNav(ros::NodeHandle& node_handler);
    HillyNav();
    /**
     *  Destroy the Agribot V S Node Handler object
     */
    virtual ~HillyNav();

    // Functions
    void imagergbCallback(const sensor_msgs::ImageConstPtr& msg);

    void findCentroids(cv::Mat pred_img);
    void Controller();
    void bestApproximate(cv::Mat seg_img, std::vector<int> centroids_x,std::vector<int> centroids_y);
    cv::Point2f camera2image(cv::Point2f& xc, cv::Mat img);
    float compute_Theta(cv::Point2f& P, cv::Point2f& Q);
    float wrapToPi(float angle);
    void IBVS();
   // std::stringstream ss;
   //
   // std::string name = "test_";
   // std::string type = ".png";
   // int ct = 0;
   // std::vector<double> ang_vel;
};
