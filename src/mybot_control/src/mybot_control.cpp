// Esse programa lê a posição atual do robô diferencial e envia comandos
// para movê-lo para uma posição desejada.
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

// Definir algumas variáveis globais.

// Inicializar as coordenadas do objetivo com valores arbitrários.
double goalX = 8.0;
double goalY = 5.0;

// Variáveis para armazenar os comandos de velocidade.
double velLinear = 0.0;
double velAngular = 0.0;


// A função callback que será executada cada vez que chegar
// uma nova mensagem.
void odomCallback(const nav_msgs::Odometry& msg)
{
	// A pose atual do robô é dada pelas seguintes variáveis:
	double x = msg.pose.pose.position.x;
	double y = msg.pose.pose.position.y;
	double theta = tf::getYaw( msg.pose.pose.orientation );
	
	// Comparar a posição atual com o objetivo.
	double d = sqrt( (goalX - x) * (goalX - x) + (goalY - y) * (goalY -y) );
	
	// Se forem diferentes, significa que o robô ainda
	// não atingiu o objetivo, e deve se mover.
	ROS_INFO_STREAM("\nPosicao atual ( x: " << x << ", y: " << y << ", theta: " << theta << " )"
	                 << "\nDestino ( x: " << goalX << ", y: " << goalY << " )");
	
	if ( d > 0.0001 )
	{
		// Decidir quais valores de velocidade o robô deve
		// assumir para se mover em direção ao objetivo
		velLinear = (d > 1.0) ? 1.0 : d;
		velAngular = theta - atan2( goalY - y, goalX - x);	
		//velAngular = 0.01;	
	}
	else 
	{
		// Se forem iguais, o robô já está no objetivo e deve parar
		velLinear = 0.0;
		velAngular = 0.0;
	}
	
	ROS_INFO_STREAM("\nComando de velocidade ( linear: " << velLinear << ", angular: " << velAngular);
}

void goalCallback(const geometry_msgs::Point& msg)
{
	goalX = msg.x;
	goalY = msg.y;
}

int main( int argc, char** argv )
{
	// Inicializar o ROS e se tornar um nó.
	ros::init(argc, argv, "mybot_control");
	ros::NodeHandle nh;
	
	// Criar um objeto publisher.
	ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>
		( "mybot/cmd_vel", 1000 );
		
	// Criar um objeto subscriber.
	ros::Subscriber odomSub = nh.subscribe("mybot/odom", 1000, 
		&odomCallback);
		
	ros::Subscriber goalSub = nh.subscribe("mybot/goal", 100,
		&goalCallback);
	
	// Executar um loop de 100Hz até que o nó seja desativado.
	ros::Rate rate(100);
	while( ros::ok() )
	{
		// Criar e preencher a mensagem de comando de velocidade.
		geometry_msgs::Twist msg;
		msg.linear.x = velLinear;
		msg.angular.z = velAngular;
		
		// Publicar a mensagem.
		cmdVelPub.publish( msg );
		
		// Passar o controle para o ROS. Ele vai executar a 
		// função callback uma vez para cada mensaem recebida.
		ros::spinOnce();
			
		// Esperar até que seja hora de executar novamente o loop.
		rate.sleep();
	}
}








