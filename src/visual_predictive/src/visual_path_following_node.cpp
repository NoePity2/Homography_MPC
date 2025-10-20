#include "utils.h"
#include "vision_utils.h"
#include <Eigen/Dense>
#include <visp3/vision/vpHomography.h>
#include <visp3/core/vpEigenConversion.h>

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
#include <std_msgs/Int32.h>
#include <eigen_conversions/eigen_msg.h>

cv::Mat refImage, currentImage;
geometry_msgs::Twist vel;     //Crea objeto "vel" para mensajes tipo "geometry_msgs/Twist"
double t0,t;
int task;
int Nreferences;

std::vector<cv::Mat> readSequentialImages(const std::string& directory) {
    std::vector<cv::Mat> images;
    int index = 0;
    
    while (true) {
        std::string filename = directory + "/reference_image" + std::to_string(index) + ".jpg";
        cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);
        if (img.empty()) {
            break;  // No more images to read
        }
        images.push_back(img);
        index++;
    }

    if (images.empty()) {
        std::cerr << "No se encontraron imágenes en el directorio " << directory << std::endl;
    }
    std::cout << "No ref images: " << images.size() << std::endl;
    Nreferences = images.size();

    return images;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cvPtr;
    try
    { 
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvPtr->image.copyTo(currentImage);
        if (currentImage.empty()) {
            ROS_ERROR("Received empty image");
        }
    }
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
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
    //Matrix_t H(3,3); // Homography matrix
    Matrix_t H_e(3,3);
    vpHomography H; //Homography matrix for visp
    Matrix_t R(3,3); // Rotation between current frame and the reference frame
    Vector3D_t t;    // Translation between current frame and the reference frame
    Vector3D_t n;    // normal vector in the reference plane
    FPTYPE d, phi;
    homographySolution homSolution;
    int counter = 0;
    int cont2 = 0, tecla;
    bool fin = false;
    std::vector<cv::Mat> desiredImages;

    K << 544.9615478515625,         0.0      , 312.6927185058594, 
         0.0              , 544.9615478515625, 238.39773559570312, 
         0.0              ,         0.0      ,   1.0;


    // Leer todas las imágenes secuenciales del directorio de referencia
    std::string REF_IMAGE_NAME = "/home/axel/Homography_MPC/src/visual_predictive/src/images_reference/";
    desiredImages = readSequentialImages(REF_IMAGE_NAME);

    if (desiredImages.empty()) {
        std::cerr << "No se pudieron leer las imágenes de referencia" << std::endl;
        return 1;
    }
    else{
        std::cout << "Ref image is ready: " << REF_IMAGE_NAME << std::endl;
    }

    task = 0;
    // Asignar la primera imagen del vector a refImage
    refImage = desiredImages[task];


    ros::init(argc, argv, "visual_path_following_node");
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
    ros::Publisher task_pub         = nh.advertise<std_msgs::Int32>("/n_task",100);
    ros::Publisher Nreferences_pub  = nh.advertise<std_msgs::Int32>("/n_references",100);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, imageCallback);
    

    std_msgs::Int32 task_msg;
    std_msgs::Int32 Nreferences_msg;

    //se necesita suscribirse al topico del error para hacer cambio de imagen de referencia

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
              task_msg.data = task;
              Nreferences_msg.data = Nreferences;
              std::cout << "current image is ready: "  << std::endl;
              cv::imwrite("/home/axel/Homography_MPC/src/visual_predictive/src/current_image.jpg",currentImage);

            //------- Estimate the homography ----------------------//
              EstimateHomography(refImage, currentImage, K, H_e, counter );
              //EstimateHomographyVisp(refImage, currentImage, K, H, counter);
              //vp::visp2eigen(H, H_e);

              //std::cout << H << '\n' << H_e << std::endl;

              RecoverFromHomography(H_e,R,t,n,d, counter, homSolution);

              phi = YRotationAngleFromRotationMatrix(R);
              std::cout << "t : " << t << std::endl; 
              std::cout << "phi : " << phi*(180/3.141592) << std::endl; 
              tf::matrixEigenToMsg(H_e, H_msg);
              tf::matrixEigenToMsg(t, ck_t_ct_msg);
              tf::matrixEigenToMsg(n, n_msg);
              homography_pub.publish(H_msg);
              ck_t_ct_pub.publish(ck_t_ct_msg);
              n_pub.publish(n_msg);
              d_msg.data = d;
              d_pub.publish(d_msg);
              phi_msg.data = phi;
              phi_pub.publish(phi_msg);
              if (abs(H_e(0, 0)) > 0.94 && abs(H_e(0, 1)) < 0.06 && abs(H_e(0, 2)) < 0.06 &&
              abs(H_e(2, 0)) < 0.06 && abs(H_e(2, 1)) < 0.06 && abs(H_e(2, 2)) > 0.94){
                task = task + 1;
                refImage = desiredImages[task];
              }

              task_pub.publish(task_msg);
              Nreferences_pub.publish(Nreferences_msg);
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
