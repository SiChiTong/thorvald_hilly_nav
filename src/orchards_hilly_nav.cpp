#include "orchards_hilly_nav.h"
#include <typeinfo>

HillyNav::HillyNav(){

  if(!readRUNParmas(nodeHandle_)){
    ROS_ERROR("Params not found!!");
  }

  // Subscribers
  pred_img_sub = nodeHandle_.subscribe("predicted_image", 1, &HillyNav::predimagergbCallback, this);

  rgb_img_sub = nodeHandle_.subscribe("camera/color/image_raw", 1, &HillyNav::imagergbCallback, this);

  imu_sub = nodeHandle_.subscribe("imu_data", 1000, &HillyNav::chatterCallback, this);

  // Publishers
  rgb_img_pub = nodeHandle_.advertise<sensor_msgs::Image>("fitted_image", 10);  // control;

  cmd_velocities = nodeHandle_.advertise<geometry_msgs::Twist>("nav_vel", 10);  // control;

  line_features = nodeHandle_.advertise<geometry_msgs::TwistStamped>("line_features", 10);  // control;

  error_features = nodeHandle_.advertise<geometry_msgs::TwistStamped>("error_features", 10);  // control;

  VelocityMsg.linear.x =0.0;
  VelocityMsg.angular.z =0.0;
}

HillyNav::~HillyNav() {
};

bool HillyNav::readRUNParmas(ros::NodeHandle& nodeHandle_){

  nodeHandle_.param("/orchards_hilly_nav/rho", rho, -0.785398/2);
  nodeHandle_.param("/orchards_hilly_nav/w_max", w_max, 0.20);
  nodeHandle_.param("/orchards_hilly_nav/hori_strips", hori_strips, 10);
  nodeHandle_.param("/orchards_hilly_nav/sensorwidth_mm", sensorwidth_mm, 90.0);
  nodeHandle_.param("/orchards_hilly_nav/fov", fov, 69.4*M_PI/180);

  nodeHandle_.param("/orchards_hilly_nav/ty", ty, 1.075);
  nodeHandle_.param("/orchards_hilly_nav/tz", tz, 1.275);

  nodeHandle_.param("/orchards_hilly_nav/fx", fx, 612.7745361328125);
  nodeHandle_.param("/orchards_hilly_nav/fy", fy, 612.6051635742188);

  nodeHandle_.param("/orchards_hilly_nav/lambda_x", lambda_x, 0.825);
  nodeHandle_.param("/orchards_hilly_nav/lambda_w", lambda_w, 1.1);

  nodeHandle_.param("/orchards_hilly_nav/controller_ID", controller_ID, 0);
  nodeHandle_.param("/orchards_hilly_nav/v", v, 0.15);

  return true;

}

void HillyNav::predimagergbCallback(const sensor_msgs::ImageConstPtr& msg){ // RGB Image
    try{
      pred_img = cv_bridge::toCvCopy(msg, "mono8")->image;
      pred_img_received=true;
     }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void HillyNav::imagergbCallback(const sensor_msgs::ImageConstPtr& msg){ // RGB Image
    try{
      rgb_img = cv_bridge::toCvCopy(msg, "rgb8")->image;
      rgb_img_received = true;
     }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("Could not convert from '%s' to 'bgr'.", msg->encoding.c_str());
    }
}

void HillyNav::chatterCallback(const sensor_msgs::Imu::ConstPtr& msg) {
   // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

   tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
   tf::Matrix3x3(q).getRPY(roll, pitch, theta_r);
}

float HillyNav::wrapToPi(float angle){
  while(angle < -M_PI && angle > M_PI){
    if(angle > M_PI){
      angle = angle - 2*M_PI;
    }else if(angle < -M_PI){
      angle = angle + 2*M_PI;
    }
  }
    return angle;
}

//*************************************************
cv::Point2f HillyNav::camera2image(cv::Point2f& xc, cv::Mat img){
  cv::Point2f xi;
  xi.x =  xc.x - image_width/2;
  xi.y =  xc.y - image_height/2;
  return xi;
}

float HillyNav::compute_Theta(cv::Point2f& P, cv::Point2f& Q){
  // compute phi
  float Y = -Q.y+P.y;
  float X = Q.x-P.x;
  float phi = atan2(Y,X);

  // compute Theta
  float Theta = wrapToPi(M_PI/2 - phi);

  return Theta;
}

// function to calculate a and b that best fit points
// represented by x[] and y[]
void HillyNav::bestApproximate(std::vector<int> centroids_x,std::vector<int> centroids_y)
{
    // Best Line Fit Parameters
    float a, b, m, c, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int n_pts = centroids_x.size();
    line_fit.clear();

    for (int i = 0; i < n_pts; i++) {
        sum_x += centroids_x[i];
        sum_y += centroids_y[i];
        sum_xy += centroids_x[i] * centroids_y[i];
        sum_x2 += pow(centroids_x[i], 2);
    }

    a = (n_pts * sum_xy - sum_x * sum_y) / (n_pts * sum_x2 - pow(sum_x, 2)); //calculate slope
    b = (sum_x2*sum_y-sum_x*sum_xy)/(sum_x2*n_pts-sum_x*sum_x);  //calculate intercept

    for(int i=0;i<n_pts;i++){
      line_fit.push_back(cv::Point(a*centroids_x[i]+b,centroids_x[i]));
    }

    // End Points of Best Approximated Line
    Q = line_fit[0];
    P = line_fit.back();
}

//*************************************************************************************************
void HillyNav::findCentroids(){

  int tmp = (image_height/2)/hori_strips;
  std::vector<int> pts_x, pts_y;
  cv::Mat white_pixels;
  cv::Scalar center;

  // Find centers based on horizontal strips
  for (int c_r = 0; c_r < hori_strips; c_r++) {

    // Horizontal Strip
    cv::Mat hori_strip = pred_img(cv::Range((image_height/2)+((c_r)*tmp),
                                    (image_height/2)+((c_r+1)*tmp)),cv::Range(0, image_width));

    cv::findNonZero(hori_strip, white_pixels);

    if ((white_pixels.total())>0) {
      center = cv::mean(white_pixels);  // Mean Strip Center

      pts_x.push_back(center[0]);
      pts_y.push_back(((image_height/2)+((c_r)*tmp)+(image_height/2)+((c_r+1)*tmp))/2);
    }

  }

  pts_x.push_back(pts_x.back());
  pts_y.push_back(pts_y.back()+(tmp/2)-1);

  // Linear Fitting to the center of the label points
  bestApproximate(pts_y, pts_x);
  std::cout << P << " " << Q << std::endl;

  // compute Theta
  float Theta = compute_Theta(P,Q);

  // compute F
  cv::Point2f _F = camera2image(P, pred_img);
  F << _F.x,
       _F.y,
       Theta;

  // compute desired F
  F_des <<  0,
            image_height/2,
            0;
}

Eigen::MatrixXf HillyNav::include_robot_model(){

  Eigen::MatrixXf Rx(3,3);

  Rx << 1, 0, 0,
        0, cos(rho), sin(rho),
        0, -sin(rho), cos(rho); // Camera pitch

// Transform robot frame to camera frame
// [xb -> zc] % [yb -> -xc] % [zb -> -yc]
  Eigen::MatrixXf Rcr(3,3);
  Rcr << 0, -1, 0,
         0, 0, -1,
         1, 0, 0; // Camera pitch

  Eigen::MatrixXf rot(3,3);

  rot = Rx*Rcr; // Rotation of the matrix to transform the coordinates of the robot to the camera

  // Translation between camera and robot in camera frame
  Eigen::MatrixXf tlc(3,1);

  //tlc << 0, tz, -ty;
  tlc << 0, ty, -tz;

  Eigen::MatrixXf tlc_fn(3,3);
  tlc_fn << 0, -tlc(2), tlc(1),
          tlc(2), 0, -tlc(0),
         -tlc(1), tlc(0), 0;

  Eigen::MatrixXf H(6,6);
  H << rot, tlc_fn,
       Eigen::MatrixXf::Zero(3,3), rot; // Adjunct matrix

  // Selection matrix to select only the desired velocites
  Eigen::MatrixXf S(3,6);
  S << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 1;

  // Robot Model
  Eigen::MatrixXf G(3,2);
  G << cos(theta_r), 0,
       sin(theta_r), 0,
       0, 1;

  return H*S.transpose()*G; // With robot model
}

// IBVS Controller using Line Features
void HillyNav::Controller(){

  float X = F(0);
  float Y = F(1);
  float Theta = F(2);

  // std::cout << X << " " << Y << " " << Theta << std::endl;

  Eigen::MatrixXf Ls(3,6);

  Ls << -fx/z_c, 0, X/z_c, (X*Y)/fy, -((pow(fx,2)-pow(X,2))/fx),  Y,
        0,   -fy/z_c, Y/z_c, (pow(fy,2)-pow(Y,2))/fy, -(X*Y)/fx, -X,
        cos(rho)*pow(cos(Theta),2)/tz, cos(rho)*cos(Theta)*sin(Theta)/tz, -(cos(rho)*cos(Theta)*(Y*sin(Theta) + X*cos(Theta)))/tz, -(Y*sin(Theta) + X*cos(Theta))*cos(Theta), -(Y*sin(Theta) + X*cos(Theta))*sin(Theta), -1;

  // compute tranformation between robot to camera frame
  Eigen::MatrixXf c(2,6);


  Eigen::MatrixXf cTR(6,2);
  if (robot_agnostic==true){
    c << 0, -sin(rho), cos(rho), 0, 0, 0,
        -ty, 0, 0, 0, -cos(rho), -sin(rho);
    cTR = c.transpose();
  }
  else{
    cTR = include_robot_model();
  }


  Eigen::MatrixXf Tv(6,1);
    Tv = cTR.col(0);

  Eigen::MatrixXf Tw(6,1);
    Tw = cTR.col(1);

  // compute Jacobian
  Eigen::MatrixXf Jv(2,6);
    Jv << Ls.row(controller_ID),
          Ls.row(2);
    Jv = Jv*Tv;

  Eigen::MatrixXf Jw(2,6);
    Jw << Ls.row(controller_ID),
          Ls.row(2);
    Jw = Jw*Tw;

    // std::cout << Jv << " " << Jw << std::endl;

  // compute control law
  Eigen::Vector2f err((F[controller_ID] - F_des[controller_ID]), wrapToPi(F[2] - F_des[2]));

  // set weights
  Eigen::MatrixXf tmp_lambda(2,1);
    tmp_lambda << lambda_x*err(0),
                  lambda_w*err(1);

  // compute control
  Eigen::MatrixXf Jw_pinv(6,2);
  Jw_pinv = Jw.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::MatrixXf w = -Jw_pinv*(tmp_lambda + Jv*v);

  w(0,0) = copysign(std::min(std::abs(w(0,0)),(float)w_max), w(0,0));

  std::cout << " " << "e_X:" << err(0) << " " << "e_theta:" << err(1) << " " << "w:" << w << std::endl;

  // Steering Commands
  // VelocityMsg.header.stamp = ros::Time::now();
  // VelocityMsg.header.frame_id = "base_link";
  VelocityMsg.linear.x = v; // Sets at constant speed
  VelocityMsg.angular.z = -w(0,0);
  cmd_velocities.publish(VelocityMsg);

  FeatureMsg.header.stamp = ros::Time::now();
  FeatureMsg.header.frame_id = "base_link";
  FeatureMsg.twist.linear.x = err(0); // Sets at constant speed
  FeatureMsg.twist.angular.z = err(1);
  line_features.publish(FeatureMsg);

  ErrorMsg.header.stamp = ros::Time::now();
  ErrorMsg.header.frame_id = "base_link";
  ErrorMsg.twist.linear.x = err(0); // Sets at constant speed
  ErrorMsg.twist.angular.z = err(1);
  error_features.publish(ErrorMsg);
}

void HillyNav::initCamParameters(){

  image_height = pred_img.rows;
  image_width = pred_img.cols;

  double f_m = ((fx*sensorwidth_mm)/image_width)/10000; // Focal Length in mm

  double rhou = (2*f_m/image_width)*tan(fov/2); // Scaling parameter in x
  double rhov = (2*f_m/image_height)*tan(fov/2); // Scaling parameter in y
  double Y_star = F_des(1)*rhov;
  z_c = tz/(sin(rho)+Y_star*cos(rho));

  // std::cout << Y_star << " " << Y_star*cos(rho) << " " << z_c << std::endl;

  CamParameters = true;
}

void HillyNav::Drawing(){

  // cv::cvtColor(pred_img, pred_img, CV_GRAY2BGR);

  // cv::circle(pred_img, cv::Point(center[0], ((image_height/2)+((c_r)*tmp)+(image_height/2)+((c_r+1)*tmp))/2),3, cv::Scalar(51, 204, 51),CV_FILLED, 8,0);

  cv::arrowedLine(rgb_img,cv::Point(line_fit.back().x,line_fit.back().y), cv::Point(line_fit[0].x, line_fit[0].y),cv::Scalar(255, 0, 0),3, CV_AA);

  // cv::imwrite("/home/vignesh/NMBU_orchard_fields/test.png", rgb_img);

  header.seq = counter; // user defined counter
  header.stamp = ros::Time::now(); // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, rgb_img);
  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  rgb_img_pub.publish(img_msg);
  counter++;


}

void HillyNav::IBVS(){

  while(ros::ok()){

    if(pred_img_received==true){

      if(CamParameters==false) initCamParameters();

      findCentroids();

      Controller();

      // Plotting
      Drawing();

      pred_img_received = false;

    } // RGB image check

    ros::spinOnce();

  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "orchards_hilly_nav");

  HillyNav nav_obj;
  nav_obj.IBVS();

  return 0;
};
