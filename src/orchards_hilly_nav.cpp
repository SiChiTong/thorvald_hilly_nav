#include "orchards_hilly_nav.h"
#include <typeinfo>

HillyNav::HillyNav(){

  // Subscribers
  pred_img_sub = nodeHandle_.subscribe("predicted_image", 1, &HillyNav::predimagergbCallback, this);

  rgb_img_sub = nodeHandle_.subscribe("overlay_image", 1, &HillyNav::imagergbCallback, this);

  // Publishers
  rgb_img_pub = nodeHandle_.advertise<sensor_msgs::Image>("fitted_image", 10);  // control;

  cmd_velocities = nodeHandle_.advertise<geometry_msgs::Twist>("nav_vel", 10);  // control;

  VelocityMsg.linear.x =0.0;
  VelocityMsg.angular.z =0.0;
}

HillyNav::~HillyNav() {
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
      rgb_img_received=true;
     }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("Could not convert from '%s' to 'bgr'.", msg->encoding.c_str());
    }
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

//*************************************************************************************************
cv::Point2f HillyNav::camera2image(cv::Point2f& xc, cv::Mat img){
  cv::Point2f xi;
  xi.x =  xc.x - img.rows/2;
  xi.y =  xc.y - img.cols/2;
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

    for(int i=0;i<n;i++){
      line_fit.push_back(cv::Point(a*centroids_x[i]+b,centroids_x[i]));
    }

    // std::cout << line_fit << std::endl;

    cv::cvtColor(seg_img,seg_img,CV_GRAY2BGR);

    cv::line(rgb_img, cv::Point(line_fit[0].x, line_fit[0].y),cv::Point(line_fit[n-1].x,
                                                            line_fit[n-1].y), cv::Scalar(255, 0, 0),1, CV_AA);

    Q = line_fit[0];
    P = line_fit[n-1];

    line_fit.clear();

    // ss<<name<<ct<<type;
    // ct++;
    //
    // std::string filename = ss.str();
    // ss.str("");
    //
    // cv::imwrite("/home/vignesh/NMBU_orchard_fields/results/test_2/"+filename,rgb_img);

    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, rgb_img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    rgb_img_pub.publish(img_msg);
    counter++;

}

//*************************************************************************************************
void HillyNav::findCentroids(cv::Mat img){

  int tmp = (img.rows/2)/hori_strips;
  std::vector<int> pts_x, pts_y;

   for (int c_r = 0; c_r < hori_strips; c_r++){

    cv::Mat topROI = img(cv::Range((img.rows/2)+((c_r)*tmp), (img.rows/2)+((c_r+1)*tmp)),cv::Range(0, img.cols));
    cv::Mat white_pixels;

    cv::findNonZero(topROI, white_pixels);

    if ((white_pixels.total())>0) {
      cv::Scalar center = cv::mean(white_pixels);
      // std::cout <<  center << std::endl;

      cv::circle(img, cv::Point(center[0], ((img.rows/2)+((c_r)*tmp)+(img.rows/2)+((c_r+1)*tmp))/2),3, cv::Scalar(51, 204, 51),CV_FILLED, 8,0);
      pts_x.push_back(center[0]);
      pts_y.push_back((img.rows/2)+((c_r+(c_r+1)/2)*tmp));
    }

  }

  // Linear Fitting to the center of the label points
  bestApproximate(img, pts_y, pts_x);

  // compute Theta
  float Theta = compute_Theta(P,Q);

  // compute F
  cv::Point2f _F = camera2image(P, pred_img);
  F << _F.x,
       _F.y,
       Theta;
  F_des <<  0,
            img.cols/2,
            0;
}


//*************************************************************************************************
void HillyNav::Controller(){

  float X = F(0);
  float Y = F(1);
  float Theta = F(2);

  Eigen::MatrixXf Ls(3,6);
  Ls << -(sin(rho)+Y*cos(rho))/tz, 0, X*(sin(rho)+Y*cos(rho))/tz, X*Y, -1-pow(X,2),  Y,
        0,   -(sin(rho)+Y*cos(rho))/tz, Y*(sin(rho)+Y*cos(rho))/tz, 1+pow(Y,2), -X*Y, -X,
        cos(rho)*pow(cos(Theta),2)/tz, cos(rho)*cos(Theta)*sin(Theta)/tz, -(cos(rho)*cos(Theta)*(Y*sin(Theta) + X*cos(Theta)))/tz, -(Y*sin(Theta) + X*cos(Theta))*cos(Theta), -(Y*sin(Theta) + X*cos(Theta))*sin(Theta), -1;

  // compute tranformation between robot to camera frame
  Eigen::MatrixXf c(2,6);
  if( controller_ID == 1){
    c << 0, -sin(rho), cos(rho), 0, 0, 0,
        -ty, 0, 0, 0, -cos(rho ), -sin(rho);
  }else{
    c << 0, sin(rho), -cos(rho), 0, 0, 0,
      -ty, 0, 0, 0, cos(rho), sin(rho);
  }

  Eigen::MatrixXf cTR(6,2);
    cTR = c.transpose();

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

  // compute control law
  Eigen::Vector2f err((F[controller_ID] - F_des[controller_ID]), wrapToPi(F[2] - F_des[2]));

  // set weights
  Eigen::MatrixXf tmp_lambda(2,1);
    tmp_lambda << 10*err(0),
                  1*err(1);

  // compute control
  Eigen::MatrixXf Jw_pinv(6,2);
  Jw_pinv = Jw.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::MatrixXf w = -Jw_pinv*(tmp_lambda + Jv*v);

  // ang_vel.push_back(w(0,0));

  std::cout << "e_X:" << err(0) << " " << "e_theta:" << err(1) << " " << "w:" << w << std::endl;

  // // Steering Commands
  // VelocityMsg.angular.z = steering_dir * w(0,0);
  // VelocityMsg.linear.x  = v;

  VelocityMsg.linear.x = v; // Sets at constant speed
  VelocityMsg.angular.z = w(0,0);
  cmd_velocities.publish(VelocityMsg);

}

void HillyNav::IBVS(){
  while(ros::ok()){

    if(pred_img_received==true){

      // cv::Mat pred_img = imread(label_file[i], 0);

      findCentroids(pred_img);

      Controller();

      pred_img_received = false;
  //
    } // RGB image check

    ros::spinOnce();

  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "orchards_hilly_nav");

  // node handler
  // ros::NodeHandle nodeHandle;
  // HillyNav vs_NodeH(nodeHandle);

  HillyNav nav_obj;
  nav_obj.IBVS();

  // // std::vector<cv::String> label_file;
  // // cv::glob("/home/vignesh/NMBU_orchard_fields/results/*.png", label_file, false);
  // //
  // // size_t count = label_file.size(); //number of png files in images folder
  // // // std::cout << count << std::endl;
  // // for (size_t i=0; i<count; i++){
  // //
  // //   std::cout<<label_file[i]<<std::endl;
  // //
  // //   cv::Mat pred_img = imread(label_file[i], 0);
  // //   nav_obj.findCentroids(pred_img);
  // //   nav_obj.Controller();
  // // }


  // // Create an output filestream object
  // std::ofstream myFile("ang_vel.csv");
  //
  // // Send the column name to the stream
  // myFile << "Steering" << "\n";
  //
  // // Send data to the stream
  // for(int i = 0; i < nav_obj.ang_vel.size(); ++i)
  // {
  //     myFile << nav_obj.ang_vel.at(i) << "\n";
  // }
  //
  // // Close the file
  // myFile.close();

  return 0;
};
