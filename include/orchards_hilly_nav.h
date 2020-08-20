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

#include <geometry_msgs/TwistStamped.h>
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

class HillyNav{

public:

    // ROS node handle.
    ros::NodeHandle nodeHandle_;

    ros::Subscriber pred_img_sub, rgb_img_sub, imu_sub;
    ros::Publisher cmd_velocities, rgb_img_pub;
    ros::Publisher line_features, error_features;

    HillyNav();

    virtual ~HillyNav();

    bool readRUNParmas(ros::NodeHandle& nodeHandle_);

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

    // Camera Params
    double rho, ty, tz, fx, fy; // Cam Tilt, Translation to robot, focal length
    double w_max;
    int hori_strips;
    double sensorwidth_mm, fov; // Opening angle
    double image_height, image_width;
    double z_c; // Depth of the pixel

    // Control Params
    double lambda_x, lambda_w;
    int controller_ID;
    double v;

    // Line Params
    std::vector<cv::Point> line_fit;
    cv::Point2f P, Q;
    Eigen::Vector3f F, F_des;

    // Robot Params
    double roll, pitch, theta_r = 0;
    geometry_msgs::TwistStamped VelocityMsg;
    geometry_msgs::TwistStamped FeatureMsg, ErrorMsg;

    cv::Mat pred_img, rgb_img;
    bool pred_img_received = false;
    bool rgb_img_received = false;
    bool CamParameters = false;
    bool robot_agnostic = true;

    std::stringstream ss;
    std::string name = "test_";
    std::string type = ".png";
    int ct = 0, counter = 0;

    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header

    sensor_msgs::Image img_msg; // >> message to be sent
   // std::vector<double> ang_vel;
};
