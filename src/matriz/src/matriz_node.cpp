/*matriz_node: usa la libreria Eigen*/

# include<ros/ros.h>
#include<iostream>
#include<Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

#ifdef __USE_SINGLE_PRECISION__
typedef float FPTYPE;
#else
typedef double FPTYPE;
#endif

typedef Eigen::Vector3d Point3D_t;
typedef Eigen::Vector3d Vector3D_t;
typedef Eigen::Vector2d Point2D_t ;

typedef Eigen::Matrix<FPTYPE, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix_t;
typedef Eigen::Transform<FPTYPE, 3, Eigen::Affine> AffineTransformation;


int main(int argc,char* argv[])
{
	ros::init(argc,argv,"matriz_node"); //Inicializa el nodo
	ros::NodeHandle nh;

	//============== Ros publishers ===============
	ros::Publisher homography_pub = nh.advertise<std_msgs::Float64MultiArray>("homograpy_numerical",100);

	ros::Rate loop_rate(10); //Se ejecuta 10 veces por segundo


    Matrix_t K(3,3); //Calibration of the camera matrix
    K << 732.21085803996016,         0.0        , 296.816303673449,
                 0.0        , 733.98760815804997, 225.60370098616005,
                 0.0        ,         0.0        ,         1.0;

	std_msgs::Float64MultiArray K_msg;
    tf::matrixEigenToMsg(K, K_msg);

	while(ros::ok()){
		homography_pub.publish(K_msg);
        ros::spinOnce();   //Activa callbacks
        loop_rate.sleep(); //Espera hasta completar el loop rate

    }


	return 0;
}