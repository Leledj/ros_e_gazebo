// Esse programa subscreve no tópico do sensor laser do robô diferencial
// e imprime na tela as medidas recebidas
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>

// Função callback que será chamada quando chegarem mensagens do sensor
void laserScanCallback(const sensor_msgs::LaserScan& msg)
{
	// msg é uma mensagem do tipo sensor_msgs/LaserScan
	
	// Essa mensagem contém informações sobre o sensor e um array
	// chamado ranges que contém as medidas de cada um dos feixes
	// de laser do sensor
	
	// Concatenar as medidas dos feixes de laser para imprimir na tela
	std::ostringstream oss;
	oss << "Ranges = [ ";
	
	// Iterar pelo array ranges, concatenando cada medida
	for (int i = 0; i < msg.ranges.size(); i++)
	{
		oss << msg.ranges[i] << " ";
	}
	
	oss << " ];";
	
	// Imprimir os valores na tela
	ROS_INFO_STREAM( oss.str() );
}

int main(int argc, char** argv)
{
	// Iniciar o ROS e criar o objeto NodeHandle
	ros::init(argc, argv, "laser_scan_sub");
	ros::NodeHandle nh;
  
	// Criar um subscriber para o tópido do sensor
  	ros::Subscriber scan_sub = nh.subscribe("mybot/scan",
		1000, &laserScanCallback);
  
	// Passar o controle para o ROS
	ros::spin();
}
