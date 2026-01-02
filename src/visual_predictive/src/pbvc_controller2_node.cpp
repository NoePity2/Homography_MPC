#include "utils.h"
#include "vision_utils.h"
#include <Eigen/Dense>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include "nav_msgs/Odometry.h"

cv::Mat refImage, currentImage;
geometry_msgs::Twist vel;     //Crea objeto "vel" para mensajes tipo "geometry_msgs/Twist"

static const std::string REF_IMAGE_NAME = "/home/axel/controller/src/pbvs_controller2/src/reference_image.jpg";
FILE *fpdata;
double t_f,t0,yaw,roll,pitch,x_k, y_k, Vel_x, W_z;

void imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  try
  {
    cv::Mat receivedImage = cv_bridge::toCvShare(msg, "bgr8")->image; // Save image in a Mat object
    currentImage = receivedImage;
    //cv::imwrite("/home/noe/catkin_ws_tutorials/src/pbvs_controller2/src/reference_image_test.jpg",currentImage);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void   odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
  /*X = msg->pose.pose.position.x;
  Y = -msg->pose.pose.position.z;
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);*/

  Vel_x = msg->twist.twist.linear.x;
  W_z =   msg->twist.twist.angular.z;
}

void pose_calback(const geometry_msgs::PoseStamped::ConstPtr &msg){
  x_k = msg->pose.position.x;
  y_k = -msg->pose.position.z;
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(pitch, yaw, roll);
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
    Matrix_t R(3,3); // Rotation between current frame and the reference frame
    Vector3D_t t;    // Translation between current frame and the reference frame
    Vector3D_t n;    // normal vector in the reference plane
    FPTYPE d;
    homographySolution homSolution;
    Matrix_t I(3,3);
    Vector3D_t u; 
    Vector3D_t Uv, Uw;
    int counter = 0;
    int cont = 0, cont2 = 0, tecla;
    bool fin = false;
    FPTYPE lambdav_ = atof(argv[2]);
    FPTYPE lambdaw_ = atof(argv[3]);


    I << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    K << 544.9615478515625,         0.0        , 312.6927185058594,
                 0.0        , 544.9615478515625  , 238.39773559570312,
                 0.0        ,         0.0        ,         1.0;

    //std::cout <<"K : " <<  K << std::endl;

    refImage = cv::imread(REF_IMAGE_NAME, cv::IMREAD_COLOR);
    if(refImage.empty())
    {
        //std::cout << "Could not read the image: " << REF_IMAGE_NAME << std::endl;
        return 1;
    }
    else{
        //std::cout << "Ref image is ready: " << REF_IMAGE_NAME << std::endl;
    }

    ros::init(argc, argv, "pbvs_controller2_node");
    ros::NodeHandle nh;
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    ros::Publisher  vel_pub;  //Crea objeto "pub" de tipo "Publisher"
    ros::Subscriber pose_sub, optitrack_sub;

    std::string topicName;
    if(argc != 1){
      topicName = argv[1];
    } 
    else
    {
      topicName = "/my_image";
      //std::cout <<"No topic name given. The default topic name will be used: " << topicName << std::endl;
    }

    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    pose_sub = nh.subscribe("/odom", 1, odomCallback);
    optitrack_sub = nh.subscribe("/natnet_ros/Turtlebot/pose", 1, pose_calback);
    vel = velocities(0,0,0);
    vel_pub.publish(vel);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, imageCallback);
    //ros::spin();
    ros::Rate loop_rate(10); //Se ejecuta 100 por segundo
    std::ofstream archivoDatos;
    //archivoDatos.open("/home/andrea/Homography_MPC/src/pbvs_controller2/datos.txt");
    //FILE* fpCd = std::fopen("/home/andrea/Homography_MPC/src/pbvs_controller2/datos.txt","w");

    fpdata = fopen("/home/axel/controller/src/pbvs_controller2/data.txt","w");

    ros::Duration(10).sleep(); //Retardo  de tiempo

    t0 = ros::Time::now().toSec();
    //std::cout >> t0 >> std::endl;
    do{
          t_f = ros::Time::now().toSec() - t0;

          if(currentImage.empty())
          {
              std::cout << "Could not read the current image: "  << std::endl;
          }
          else{
              std::cout << "current image is ready: "  << std::endl;
              //cv::imwrite("/home/noe/catkin_ws_tutorials/src/pbvs_controller2/src/current_image_1.jpg",currentImage);
              EstimateHomography(refImage, currentImage, K, H, counter );
              RecoverFromHomography(H,R,t,n,d, counter, homSolution);
              Rodriguez(R,u);
              PBVSController(R, t, u, Uv, Uw, lambdav_, lambdaw_);
              std::cout << "Calculando "  << std::endl;
              vel = velocities(Uv(2),-Uv(0),Uw(1));
              std::cout << Uv(2) << " " << Uw(1) << std::endl;
              //std::fprintf(fpCd, "%.4f\t%.4f\n",Uv(2),Uw(1));
          }
          vel_pub.publish(vel);
          if(cont==2)
          {
              //printf("vx[m/s] = %.2f\t vy[m/s] = %.2f\t wz[rad/s] %.2f \n",Uv(2),-Uv(0),Uw(1));
              //std::cout << "R = " << R <<  std::endl;
              //std::cout << "\n t = " << t <<  std::endl;
              //std::cout << "\n u = " << u <<  std::endl;
              // std::cout << "\n n = " << n <<  std::endl;
              // std::cout << "\n d = " << d <<  std::endl;
              //std::cout << "\n H = " << H <<  std::endl;
              
              //std::cout << "\n H = R*(I-t*n^T) =  " << R*(I - t*n.transpose()/d) <<  std::endl;
              cont = 0;
          }else
          {
              cont++;
          }
          // if(cont2==10)
          // {
          //   //fprintf(fpCd, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t",t,Px,Py,yaw,xd,yd);
          //   cont2 = 0;
          // }
          // else
          // {
          //   cont2++;
          // }
          counter++;
          //Write in a file
          fprintf(fpdata, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", t_f, x_k, y_k, yaw, Uv(2), Uw(1), Vel_x, W_z); // Guarda tiempo, posición y velocidades
          ros::spinOnce();   //Activa callbacks
          if(kbhit()) { tecla = getchar(); }
          if(tecla == 27){fin = true; }
          loop_rate.sleep(); //Espera hasta completar el loop rate
    }

    while(fin == false);
    vel = velocities(0,0,0);
    vel_pub.publish(vel);
    archivoDatos.close();
    fclose(fpdata); //Close the file


    //cv::destroyWindow("Video Stream");
    return 0;
}