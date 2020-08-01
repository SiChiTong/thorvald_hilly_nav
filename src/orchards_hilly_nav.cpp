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
    void bestApproximate(cv::Mat seg_img, std::vector<int> centroids_x,std::vector<int> centroids_y);


    // // private:
    // pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;

};

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

// function to calculate m and c that best fit points
// represented by x[] and y[]
void HillyNav::bestApproximate(cv::Mat seg_img, std::vector<int> centroids_x,std::vector<int> centroids_y)
{
    float a, b, m, c, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int n = centroids_x.size();
    for (int i = 0; i < centroids_x.size(); i++) {
        sum_x += centroids_x[i];
        sum_y += centroids_y[i];
        sum_xy += centroids_x[i] * centroids_y[i];
        sum_x2 += pow(centroids_x[i], 2);
    }

    a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - pow(sum_x, 2)); //calculate slope
    // c = (sum_y - m * sum_x) / n;
    b = (sum_x2*sum_y-sum_x*sum_xy)/(sum_x2*n-sum_x*sum_x);  //calculate intercept

    std::vector<int> fit_x;
    // std::vector<int> plotyc = linspace(seg_img.rows/2, seg_img.rows-1, seg_img.rows/2);
    cv::cvtColor(seg_img,seg_img,CV_GRAY2BGR);
    for(int i=0;i<centroids_x.size();i++){
      fit_x.push_back(a*centroids_x[i]+b);
    }

    cv::line(seg_img, cv::Point(fit_x[0], centroids_x[0]),cv::Point(fit_x[centroids_x.size()-1], centroids_x[centroids_x.size()-1]), cv::Scalar(51, 204, 51),1, CV_AA);

    cv::imwrite("test.png",seg_img);

    std::cout << "m =" << m;
    std::cout << "\nc =" << c;
}

//*************************************************************************************************
void HillyNav::findCentroids(cv::Mat img){

  int hori_strips = 10;
  int tmp = (img.rows/2)/hori_strips;
  std::vector<int> pts_x, pts_y;

  for (int c_r = 0; c_r < hori_strips; c_r++){

    cv::Mat topROI = img(cv::Range((img.rows/2)+((c_r)*tmp), (img.rows/2)+((c_r+1)*tmp)),cv::Range(0, img.cols));
    cv::Mat white_pixels;

    cv::findNonZero(topROI, white_pixels);
    if ((white_pixels.total())>0) {
      cv::Scalar center = cv::mean(white_pixels);
      std::cout <<  center << std::endl;

      cv::circle(img, cv::Point(center[0], ((img.rows/2)+((c_r)*tmp)+(img.rows/2)+((c_r+1)*tmp))/2),3, cv::Scalar(51, 204, 51),CV_FILLED, 8,0);
      pts_x.push_back(center[0]);
      pts_y.push_back((img.rows/2)+((c_r+(c_r+1)/2)*tmp));
    }

  }

  bestApproximate(img, pts_y, pts_x);


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
