# Introdução ao ROS e ao simulador Gazebo
Nessa pasta encontram-se os arquivos utilizados na disciplina COM920 - Tópicos Avançados - Simulação. Para utilizar esse material, siga as instruções abaixo:

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

## Máquina Virtual

Para fazer os exercícios e testes em casa, fazer o download da máquina virtual no site:

http://nootrix.com/downloads/#RosVM
