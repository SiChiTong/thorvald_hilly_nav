#include <ros/ros.h>
#include <glob.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>

class HillyNav{

  public:
    // sensor_msgs::LaserScan incoming_scans;
    // sensor_msgs::PointCloud2 ptcloud_ros;
    // size_t num_ranges;
    //
    // tf::TransformListener odom_listener;
    // tf::StampedTransform odom_transform;
    // geometry_msgs::TransformStamped odom_geom_transform;
    // geometry_msgs::PointStamped cart_pts_, cart_pts_trans_;
    // bool constants = false; // Check to read laserscan constants only once
    // double theta;
    //
    // std::string world_frame_ = "odom_combined";
    // std::string laser_frame_ = "base_laser_link";
    //
    // ros::NodeHandle nh_;
    // ros::Subscriber scan_sub;  // Subscribers
    // ros::Publisher ptcloud_pub;  // Publishers
    //
    // ConvToPtCloud(); // Constructor
    //
    // Functions
    // void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void findCentroids(cv::Mat img);
    void Controller();

    // // private:
    // pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;

};

//*************************************************************************************************
void HillyNav::findCentroids(cv::Mat img){

  // cv::Mat locations;
  // cv::findNonZero(img, locations);
  // std::cout << locations.total() << std::endl;
  // for (int c_r = 0; c_r < locations.total(); c_r++){
  //   // std::cout << "Zero#" << c_r << ": " << locations.at<cv::Point>(c_r).x << ", " << locations.at<cv::Point>(c_r).y << std::endl;
  //   cv::Point pnt = locations.at<cv::Point>(c_r);
  //   // if nonzero_x.size()!=0:
  //       //centerline_arr[c_r] = int(np.mean(nonzero_x))
  //       // cv::circle(img, (int(centerline_arr[c_r]), c_r), 5, (0, 0, 255), -1)
  // }
  int tmp = (img.cols/2)/10;
  for (int c_r = 0; c_r < 5; c_r++){
    cv::Mat topROI = img(0, (img.rows/2)+((c_r-1)*tmp), img.cols, (img.rows/2)+(c_r*tmp));

    cv::Scalar center = cv::mean(topROI);

    cv::circle(topROI, cv::Point(center[0], center[1]),3, Scalar(51, 204, 51),CV_FILLED, 8,0);

    cv::imwrite("test.png",topROI);

  }



}


//*************************************************************************************************
void HillyNav::Controller(){

  // float X = F(0);
  // float Y = F(1);
  // float Theta = F(2);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "orchards_hilly_nav");

  HillyNav nav_obj;

  std::vector<cv::String> label_file;
  cv::glob("/home/vignesh/NMBU_orchard_fields/results/*.png", label_file, false);

  size_t count = label_file.size(); //number of png files in images folder
  std::cout << count << std::endl;
  for (size_t i=0; i<count; i++){

    std::cout<<label_file[i]<<std::endl;

    cv::Mat pred_img = imread(label_file[i], 0);
    nav_obj.findCentroids(pred_img);
  }

  // conv_obj.do_conversion();

  return 0;
};
