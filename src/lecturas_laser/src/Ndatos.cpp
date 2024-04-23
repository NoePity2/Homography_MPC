#include "laser.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "lecturas");
    ros::NodeHandle n;
    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    //~ ros::spin();
    
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		//if(NUM_MEDICIONES!=0)
		//{
			//Ajuste();//SIEMPRE VA AL INICIO
			cordenadasXY();
			//Guardararchivos();
			FindGaps(0.5);// indicar el valor de la diferencia para considerar un gap
			FindObsCenter();
			//imprimirDistancia();
			//imprimirDistaciaAng(-190,190);
			imprimirGaps();
		//}
		
		
	//AL FINAL
		ros::Duration(0.1).sleep();
		ros::spinOnce();
		
	}
	
	//delete [] MedidaLazer;//manera correcta de libera memoria de arreglos puntero
    return 0;
}
