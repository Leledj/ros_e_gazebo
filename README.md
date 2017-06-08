# Introdução ao ROS e ao simulador Gazebo
Esse repositório contém arquivos a serem utilizados nas aulas de simuladores envolvendo o ROS e o Gazebo. 

Uma única vez, execute os seguintes comandos:
```
echo "source /home/viki/catkin_ws/devel/setup.bash" > ~/.bashrc
source ~/.bashrc
```

No início de cada aula, baixar e compilar os pacotes executando os seguintes comandos:
```
cd ~
sudo rm -r ros_e_gazebo
git clone https://github.com/viscap/ros_e_gazebo
rm -r ~/catkin_ws/src
cp -r ~/ros_e_gazebo/src ~/catkin_ws/
cd ~/catkin_ws
catkin_make
```

Link para fazer download da máquina virtual:

http://nootrix.com/downloads/#RosVM
