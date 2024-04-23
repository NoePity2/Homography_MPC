#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <fstream>                                     // abrir,cerrar,guaradar 
#define RAD2DEG(x) ((x)*180./M_PI)


using namespace std; 
using sensor_msgs::LaserScan;
ofstream archivo;

ros::Subscriber sub;


typedef struct
{
	double distancia;//Distancia medida desde el laser a un obstaculo
	double angulo;//Angulo de la medicion de distancia
	double Radianes;//Angulo en radianes
	double x;//coordenada x de la medicion
	double y;//coordenada y de la medicion
	bool gap;//Evaluar si el rayo es un gap, Toma el rayo con mayor distancia como el gap
} Mediciones;

typedef struct
{
	//indices de gaps que delimitan el obstaculo
	int indiceOne;
	int indiceTwo;
	//coordenadas del centro del obstacilo
	double Cx;
	double Cy;
	double distancia;//distancia del laser al centro del obstaculo
}Landmark;

// Mediciones *MedidaLazer;
// int contador = 0;// Guarda el numero de mediciones del láser
const int NUM_MEDICIONES = 360;
Mediciones MedidaLazer[NUM_MEDICIONES];
Landmark Obstaculo;


//calcular la coordenada del centro del obstaculo y la distancia desde el laser al obstaculo
void FindObsCenter(void)
{
	//Calculo de las coordenadas del centro del obstaculo: Cx=(x2+x1)/2   Cy=(y2+y1)/2
	Obstaculo.Cx=(MedidaLazer[Obstaculo.indiceOne].x+MedidaLazer[Obstaculo.indiceTwo].x)/2;
	Obstaculo.Cy=(MedidaLazer[Obstaculo.indiceOne].y+MedidaLazer[Obstaculo.indiceTwo].y)/2;
	
	//Calculo de la distancia al obstaculo:  sqrt((x2-x1)^2  +  (y2-y1)^2)
	Obstaculo.distancia=sqrt(Obstaculo.Cx*Obstaculo.Cx + Obstaculo.Cy*Obstaculo.Cy);
	cout<<"Distancia al obstaculo: "<<Obstaculo.distancia<<endl;
	cout<<"Centro del Obstaculo: X: "<<Obstaculo.Cx<<"\t Y: "<<Obstaculo.Cy<<endl;
}

void FindGaps(double ValDif)
{
	int i;

	for(i= 1; i< NUM_MEDICIONES;i++)
	{
		//si la diferencia entre 2 distancias de los rayos es mayor que un cierto valor
		if( abs(MedidaLazer[i-1].distancia-MedidaLazer[i].distancia)>ValDif)
		{
			//Detecta que la medición anterior tiene el gap
			if( (MedidaLazer[i-1].distancia>MedidaLazer[i].distancia))
			{
				MedidaLazer[i-1].gap=1;//Medicion anterior tiene el gap
				Obstaculo.indiceOne=i;//indice del rayo que si toca el objeto
			}
			//Detecta qe la medicion actual tiene el gap	
			else //if(MedidaLazer[i-1].distancia<MedidaLazer[i].distancia)
			{
				MedidaLazer[i].gap=1;//Medicion actual tiene el gap
				Obstaculo.indiceTwo=i-1;//indice del rayo que si toca el objeto
			}
		}
	}
}

void imprimirDistancia()
{
	cout<< "++++++++++++++++INICIO: "<<endl;
	int i;
	for(i= 0; i< NUM_MEDICIONES;i++)
	{

		cout<< "rayo ["<<i<<"]:"<<endl;
		cout<< "distacia: " <<MedidaLazer[i].distancia<<endl;
		cout<< "Angulo: " <<MedidaLazer[i].angulo<<endl;
		cout<< "Radianes: " <<MedidaLazer[i].Radianes<<endl;
		cout<< "gap: " <<MedidaLazer[i].gap<<endl;
		cout<< "cordenada x: "<<MedidaLazer[i].x<<endl;
		cout<< "cordenada y: "<<MedidaLazer[i].y<<endl;

	}
	cout<< "++++++++++++++++++++++FIN: "<<endl;
	
}
	 
void imprimirDistaciaAng(double minDeg, double maxDeg)
{
	cout<< "++++++++++++++++INICIO: "<<endl;
	int i;
	for(i= 0; i< NUM_MEDICIONES;i++)
	{
		if(MedidaLazer[i].angulo<=maxDeg  && MedidaLazer[i].angulo>=minDeg)
		{
			cout<< "rayo ["<<i<<"]:"<<endl;
			cout<< "distacia: " <<MedidaLazer[i].distancia<<endl;
			cout<< "Angulo: " <<MedidaLazer[i].angulo<<endl;
			cout<< "Radianes: " <<MedidaLazer[i].Radianes<<endl;
			cout<< "gap: " <<MedidaLazer[i].gap<<endl;
			cout<< "cordenada x: "<<MedidaLazer[i].x<<endl;
			cout<< "cordenada y: "<<MedidaLazer[i].y<<endl;
		}
	}
	cout<< "++++++++++++++++++++++FIN: "<<endl;
	
}



void imprimirGaps()
{
	cout<< "INICIO: "<<endl;
	int i=0;
	for(i= 0; i< NUM_MEDICIONES;i++)
	{
		if(MedidaLazer[i].gap)
		{
			cout<< "rayo ["<<i<<"]:"<<endl;
			cout<< "distacia: " <<MedidaLazer[i].distancia<<endl;
			cout<< "Angulo: " <<MedidaLazer[i].angulo<<endl;
			cout<< "gap: " <<MedidaLazer[i].gap<<endl;
		}			
	}
	cout<< "FIN: "<<endl;

}

void Ajuste(void)
{
	for(int i = 0; i < NUM_MEDICIONES; i++) 
    {
		
       if(MedidaLazer[i].distancia<=0.1) 
       {
		   if(i!=0)
		   {
				MedidaLazer[i].distancia=MedidaLazer[i-1].distancia;  
			}
			else
			{
				MedidaLazer[i].distancia=MedidaLazer[i+1].distancia;  
			}
	   }
   }
}


void cordenadasXY(void)


{
	int i;

	    for(int i = 0; i < NUM_MEDICIONES; i++) 
		{ 
			MedidaLazer[i].x = MedidaLazer[i].distancia*cos(MedidaLazer[i].Radianes);
			MedidaLazer[i].y = MedidaLazer[i].distancia*sin(MedidaLazer[i].Radianes);

		}
	
	
}

void Guardararchivos(void)

{
	archivo.open("Datos.csv");

	int i;

	    for(int i = 0; i < NUM_MEDICIONES; i++) 
		{ 
			archivo<< MedidaLazer[i].x<<","<<MedidaLazer[i].y<<endl;
		}
	
	archivo.close();
}



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	
    //contador = scan->scan_time / scan->time_increment;// numero de mediciones
    //MedidaLazer = new Mediciones[contador];
    double rad;
    double degree;
  
    
	
    for(int i = 0; i < NUM_MEDICIONES; i++) //Guardado de todas las mediciones Laser en variable manipulable MedidaLazer
    {
		rad=scan->angle_min + scan->angle_increment * i;
		
		degree = RAD2DEG(rad);//obtener el ángulo de cada medición
		MedidaLazer[i].distancia=scan->ranges[i];
		MedidaLazer[i].angulo = degree;
		MedidaLazer[i].Radianes=rad;
		MedidaLazer[i].gap=0;
   }
}


