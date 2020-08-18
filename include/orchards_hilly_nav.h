#include <ros/ros.h>
#include <glob.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <Eigen/Geometry>

#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

class HillyNav{


public:

    double rho = 0.785398;
    double ty = 0.735;
    double tz = 1.205;
    float w_max = 0.2;
    int hori_strips = 10;
    double fx = 612.7745361328125;
    double fy = 612.6051635742188;
    double sensorwidth_mm = 90;
    double fov = 69.4*M_PI/180; // Opening angle

    std::vector<cv::Point> line_fit;
    cv::Point2f P;
    cv::Point2f Q;
    Eigen::Vector3f F;
    Eigen::Vector3f F_des;
    int controller_ID = 0;
    double v = 0.4;
    double roll, pitch, theta_r = 0;
    geometry_msgs::Twist VelocityMsg;
    double image_height, image_width;

    // Camera Parameters
    double z_c;

    cv::Mat pred_img, rgb_img;
    bool pred_img_received = false;
    bool rgb_img_received = false;
    bool CamParameters = false;
    bool robot_agnostic = true;

    // ROS node handle.
    ros::NodeHandle nodeHandle_;

    ros::Subscriber pred_img_sub, rgb_img_sub, imu_sub;
    ros::Publisher cmd_velocities, rgb_img_pub;

    HillyNav();

    virtual ~HillyNav();

    // Functions
    void predimagergbCallback(const sensor_msgs::ImageConstPtr& msg);
    void imagergbCallback(const sensor_msgs::ImageConstPtr& msg);
    void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg);

    void findCentroids();
    void Controller();
    void bestApproximate(std::vector<int> centroids_x,std::vector<int> centroids_y);
    cv::Point2f camera2image(cv::Point2f& xc, cv::Mat img);
    float compute_Theta(cv::Point2f& P, cv::Point2f& Q);
    float wrapToPi(float angle);
    void IBVS();
    void initCamParameters();
    void Drawing();
    Eigen::MatrixXf include_robot_model();

    std::stringstream ss;

    std::string name = "test_";
    std::string type = ".png";
    int ct = 0, counter = 0;

    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header

    sensor_msgs::Image img_msg; // >> message to be sent
   // std::vector<double> ang_vel;
};
