#include "utils.h"
#include "vision_utils.h"
#include <Eigen/Dense>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdlib.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <eigen_conversions/eigen_msg.h>

#include <visp3/vision/vpHomography.h>
#include <visp3/core/vpEigenConversion.h>

cv::Mat refImage, currentImage;
geometry_msgs::Twist vel;     //Crea objeto "vel" para mensajes tipo "geometry_msgs/Twist"
double t0,t;

static const std::string REF_IMAGE_NAME = "/home/noe/ddr_ws/src/visual_predictive/src/reference_image.jpg";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cvPtr;

	try
	{ 
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
  cvPtr->image.copyTo(currentImage);
}

geometry_msgs::Twist velocities(float vx, float vy, float w){
  vel.linear.x  = vx;
  vel.linear.y  = vy;
  vel.angular.z = w;
  return(vel);
}

int main(int argc, char **argv)
{
    Matrix_t K(3,3); //Calibration of the camera matrix
    Matrix_t H(3,3); // Homography matrix
    vpHomography Hvisp;
    Matrix_t R(3,3); // Rotation between current frame and the reference frame
    Vector3D_t t;    // Translation between current frame and the reference frame
    Vector3D_t n;    // normal vector in the reference plane
    FPTYPE d, phi;
    FPTYPE d_fix = 5;
    homographySolution homSolution;
    int counter = 0;
    int cont2 = 0, tecla;
    bool fin = false;

    // K << 522.1910329546544,         0.0      , 320.5, 
    //      0.0              , 522.1910329546544, 240.5, 
    //      0.0              ,         0.0      ,   1.0;

    // Calibration camera matrix of turtlebot simulator
    K << 402.2965906510784,        0.0       , 320.5, 
                 0.0      , 402.2965906510784, 240.5, 
                 0.0      ,        0.0       ,   1.0;


    refImage = cv::imread(REF_IMAGE_NAME, cv::IMREAD_COLOR);
    if(refImage.empty())
    {
        std::cout << "Could not read the image: " << REF_IMAGE_NAME << std::endl;
        return 1;
    }
    else{
        std::cout << "Ref image is ready: " << REF_IMAGE_NAME << std::endl;
    }

    ros::init(argc, argv, "visual_predictive_node");
    ros::NodeHandle nh;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    
    std::string topicName;
    if(argc != 1){
      topicName = argv[1];
    } 
    else
    {
      topicName = "/my_image";
      std::cout <<"No topic name given. The default topic name will be used: " << topicName << std::endl;
    }

    ros::Publisher homography_pub   = nh.advertise<std_msgs::Float64MultiArray>("/homograpy_numerical",100);
    ros::Publisher ck_t_ct_pub      = nh.advertise<std_msgs::Float64MultiArray>("/ck_t_ct",100);
    ros::Publisher n_pub            = nh.advertise<std_msgs::Float64MultiArray>("/n_plane_vector",100);
    ros::Publisher d_pub            = nh.advertise<std_msgs::Float64>("/d_value",100);
    ros::Publisher phi_pub          = nh.advertise<std_msgs::Float64>("/phi",100);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, imageCallback);

    //ros::spin();
    ros::Rate loop_rate(10); //Se ejecuta 100 por segundo
    ros::Duration(2).sleep(); //Retardo  de tiempo

    std_msgs::Float64MultiArray H_msg,ck_t_ct_msg,n_msg;
    std_msgs::Float64 d_msg,phi_msg;

    do{
          if(currentImage.empty())
          {
              std::cout << "Could not read the current image: "  << std::endl;
          }
          else{
              //std::cout << "current image is ready: "  << std::endl;
              cv::imwrite("/home/noe/ddr_ws/src/visual_predictive/src/current_image.jpg",currentImage);
              EstimateHomography(refImage, currentImage, K, H, counter );
              //EstimateHomographyVisp(refImage, currentImage, K, Hvisp, counter );
              //vp::visp2eigen(Hvisp,H);

              RecoverFromHomography(H,R,t,n,d, counter, homSolution);
              phi = YRotationAngleFromRotationMatrix(R);
              //std::cout << "t : " << t << std::endl; 
              std::cout << "H : " << H << std::endl;
              std::cout << "d_estimate : " << d << std::endl;
              std::cout << "phi : " << phi*(180/3.141592) << std::endl; 
              tf::matrixEigenToMsg(H, H_msg);
              tf::matrixEigenToMsg(t, ck_t_ct_msg);
              tf::matrixEigenToMsg(n, n_msg);
              homography_pub.publish(H_msg);
              ck_t_ct_pub.publish(ck_t_ct_msg);
              n_pub.publish(n_msg);
              d_msg.data = d;
              d_pub.publish(d_msg);
              phi_msg.data = phi;
              phi_pub.publish(phi_msg);

          }
          counter++;
          ros::spinOnce();   //Activa callbacks
          if(kbhit()) { tecla = getchar(); }
          if(tecla == 27){fin = true; }
          loop_rate.sleep(); //Espera hasta completar el loop rate
    }while(fin == false);
    //vel = velocities(0,0,0);
    //pub.publish(vel);


    //cv::destroyWindow("Video Stream");
    return 0;
}
