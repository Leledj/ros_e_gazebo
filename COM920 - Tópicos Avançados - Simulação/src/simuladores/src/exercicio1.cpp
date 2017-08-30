#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iomanip> 

// Definir algumas variáveis globais.

// Inicializar as coordenadas do objetivo com valores arbitrários.
double goalX = ;
double goalY = ;

// Variáveis para armazenar os comandos de velocidade.
double velLinear = 0.0;
double velAngular = 0.0;


// A função callback que será executada cada vez que chegar
// uma nova mensagem.
void poseMessageReceived(const turtlesim::Pose& msg)
{
	// Os campos x, y e theta da mensagem recebida representam
	// a posição atual da tartaruga.
	
	// Comparar a posição atual com o objetivo.
	
	// Se forem diferentes, significa que a tartaruga ainda
	// não atingiu o objetivo, e deve se mover.
	
	// Decidir quais valores de velocidade a tartaruga deve
	// assumir para se mover em direção ao objetivo
	velLinear = ;
	velAngular = ;
}


int main( int argc, char** argv )
{
	// Inicializar o ROS e se tornar um nó.
	ros::init(argc, argv, "exercicio1");
	ros::NodeHandle nh;
	
	// Criar um objeto publisher.
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>
		( "turtle1/cmd_vel", 1000 );
		
	// Criar um objeto subscriber.
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, 
		&poseMessageReceived);
	
	// Semente para o gerador de números aleatórios.
	srand(time(0));
	
	// Executar um loop de 2Hz até que o nó seja desativado.
	ros::Rate rate(2);
	while( ros::ok() )
	{
		// Criar e preencher a mensagem de comando de velocidade.
		geometry_msgs::Twist msg;
		msg.linear.x = velLinear;
		msg.angular.z = velAngular;
		
		// Publicar a mensagem.
		pub.publish( msg );
		
		// Passar o controle para o ROS. Ele vai executar a 
		// função callback uma vez para cada mensaem recebida.
		ros::spinOnce();
			
		// Esperar até que seja hora de executar novamente o loop.
		rate.sleep();
	}
}








