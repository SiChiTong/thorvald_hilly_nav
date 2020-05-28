#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> /*doTransform*/

class ConvToPtCloud{

  public:
    sensor_msgs::LaserScan incoming_scans;
    sensor_msgs::PointCloud2 ptcloud_ros;
    size_t num_ranges;

    tf::TransformListener odom_listener;
    tf::StampedTransform odom_transform;
    geometry_msgs::TransformStamped odom_geom_transform;
    geometry_msgs::PointStamped cart_pts_, cart_pts_trans_;
    bool constants = false; // Check to read laserscan constants only once
    double theta;

    std::string world_frame_ = "odom_combined";
    std::string laser_frame_ = "base_laser_link";

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub;  // Subscribers
    ros::Publisher ptcloud_pub;  // Publishers

    ConvToPtCloud(); // Constructor

    // Functions
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void do_conversion();

    // private:
    pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
};

ConvToPtCloud::ConvToPtCloud(){

  // Subscribers
  scan_sub = nh_.subscribe("base_scan", 100, &ConvToPtCloud::scanCallback, this);

  // Publishers
  ptcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("converted_pointclouds", 10);

  // Wait for transform
  odom_listener.waitForTransform(world_frame_, laser_frame_, ros::Time(), ros::Duration(1.0));
}

// Laser Scan data
void ConvToPtCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {

  num_ranges = scan_msg->ranges.size(); // Size of the laserscans

  if(num_ranges > 0){ // Check if laserscans are received

    incoming_scans.header.stamp = scan_msg->header.stamp;
    incoming_scans.ranges = scan_msg->ranges;
    incoming_scans.intensities = scan_msg->intensities;

    if(!constants){
      incoming_scans.angle_min = scan_msg->angle_min;
      incoming_scans.angle_max = scan_msg->angle_max;
      incoming_scans.angle_increment = scan_msg->angle_increment;

      // PointCloud Constant Variables
      ptcloud_pcl.header.frame_id = world_frame_;
      ptcloud_pcl.height = 1;
      ptcloud_pcl.width = num_ranges;
      ptcloud_pcl.points.resize(num_ranges);

      constants = true;
    }

    // TF2 listener to convert from laser frame into world frame
    try{
    odom_listener.lookupTransform(world_frame_, laser_frame_, ros::Time(0), odom_transform);
    }
    catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
    }

    tf::transformStampedTFToMsg(odom_transform, odom_geom_transform); // convert to geometry_msgs Transform (for doTransform)

    // Converting into cartesian coordinates (3D)
    for (int r_ind = 0; r_ind <= num_ranges; r_ind++) {
     theta = incoming_scans.angle_min + r_ind * incoming_scans.angle_increment;
     cart_pts_.point.x = incoming_scans.ranges[r_ind]*cos(theta);
     cart_pts_.point.y = incoming_scans.ranges[r_ind]*sin(theta);

     tf2::doTransform(cart_pts_, cart_pts_trans_, odom_geom_transform);  // Switch from Laser frame to World Frame

     // Assign the transformed values in pcl::PointCloud<pcl::PointXYZI>
     ptcloud_pcl.points[r_ind].x = cart_pts_trans_.point.x;
     ptcloud_pcl.points[r_ind].y = cart_pts_trans_.point.y;
     if(incoming_scans.intensities.size()>0)
        ptcloud_pcl.points[r_ind].intensity = incoming_scans.intensities[r_ind];

    }

    // pcl::PointCloud<pcl::PointXYZI> into sensor_msgs::PointCloud2
    pcl::toROSMsg(ptcloud_pcl, ptcloud_ros);

    // Publish the sensor_msgs::PointCloud2 type
    ptcloud_pub.publish(ptcloud_ros);

  }
  else{
    ROS_ERROR("No laser scans received");
  }

}

// Call the subscriber here and do all the calculation here.
void ConvToPtCloud::do_conversion()
{

  ros::Rate r(10);
  while(ros::ok()){

    ros::spinOnce();

    r.sleep(); // Publish the topic at 10Hz
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "BOR_node_test");

  ConvToPtCloud conv_obj;
  conv_obj.do_conversion();

  return 0;
};
