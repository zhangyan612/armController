# Robot Arm Control

1. change value in config
2. run the startServer.bat file to start the socket server
3. run arm state file and armMQ file
4. run servo state file


rabbitmq service install on windows server for pub sub communication

https://www.rabbitmq.com/install-windows.html#configure
https://www.rabbitmq.com/install-windows.html

Windows open port 5672 for rabbit mq 
https://pureinfotech.com/open-port-firewall-windows-10/

log and config location
C:\Users\username\AppData\Roaming\RabbitMQ
C:\Program Files\RabbitMQ Server\rabbitmq_server-3.12.4\sbin
%APPDATA%\RabbitMQ\rabbitmq.conf

windows local machine 192.168.0.195

rabbitmq-plugins enable rabbitmq_management
log in to below site to add a user 
http://localhost:15672

rabbitmq-service start
rabbitmq-service stop


rabbitmqctl add_user 'yan' 'yan'
rabbitmqctl list_users

Install the "URL Rewrite" module if it's not already installed.
https://www.iis.net/downloads/microsoft/url-rewrite
https://iis-umbraco.azurewebsites.net/downloads/microsoft/application-request-routing

Create a new "Inbound Rule" to rewrite WebSocket requests. Here's a sample rule:

Match URL: Requested URL: Matches the Pattern
Using: Wildcards
Pattern: ws://localhost:7890/*
Action: Rewrite
Rewrite URL: ws://127.0.0.1:7890/{R:0}
Append query string: Checked
This rule will redirect WebSocket requests from yourdomain.com to your Python WebSocket server running on 127.0.0.1:8765.

# base robot config on port
ifconfig

echo $ROS_MASTER_URI
echo $ROS_HOSTNAME

export ROS_HOSTNAME=192.168.0.177
export ROS_HOSTNAME=192.168.1.184

sudo nano .bashrc
or
gedit .bashrc



# rosbridge for moving control

ROS Update, remote server control

1. Install rosbridge
sudo apt-get install ros-melodic-rosbridge-server

echo $ROS_MASTER_URI
echo $ROS_HOSTNAME
ifconfig

export ROS_HOSTNAME=192.168.0.177
export ROS_HOSTNAME=192.168.1.184

sudo nano .bashrc
or
gedit .bashrc


#to make it permanent, add the line to this file
source ~/.bashrc

roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

roslaunch rosbridge_server rosbridge_websocket.launch

rostopic list
rostopic echo /topic_name
rostopic echo /PowerVoltage
rostopic type /PowerVoltage

rostopic echo /odom
rostopic echo /imu

13.WEB浏览器显示摄像头
主机：roslaunch usb_cam usb_cam-test.launch
          rosrun web_video_server web_video_server
主机网页查看：http://localhost:8080/ (发出热点的为主机)
客户机网页查看：http://192.168.0.100:8080 (连接热点的为客户机)

新版启动相机命令为以下
roslaunch turn_on_wheeltec_robot wheeltec_camera.launch


# Whisper web
download vad model 
cd ~/.cache/whisper-live/
sudo wget -O silero_vad.onnx https://github.com/snakers4/silero-vad/raw/master/files/silero_vad.onnx


 cd voice\whisperWeb\ui
 python -m http.server


arecord -d 10 -r 48000 -c 2 -f S16_LE test.wav

arecord -f S16_LE -c1 -r 16000 -t raw -D default | nc localhost 6006

pyaudio install
sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
sudo apt-get install ffmpeg libav-tools
sudo pip install pyaudio


# helpful
ROS开发教程
ROS 系统里面的参数类似于单片机开发中的全局变量，由ROS Master 管理，其通信机制较为简单，不涉及 TCP/UDP 通信。我们可以使用“rosparam”命令去查看或者设置机器人的参数
rosparam list 
rosparam get /wheeltec_robot/serial_baud_rate
rosparam set /wheeltec_robot/serial_baud_rate 9600 -- 修改参数 临时
如果我们需要永久修改某一个参数，需要通过修改源码，也就是launch或者 yaml 等文件里面的参数

查看机器人的 tf 树， 坐标变换关系
rosrun rqt_tf_tree rqt_tf_tree
base_link 与机器人中心重合

conda create -n robot python=3.9
conda remove --name robot --all

conda create -n vision python=3.7

Add comment to EOF to disable conda or remove comment to enable it
base robot only works when it's disabled 

gedit .bashrc
source ~/.bashrc

:<<EOF
source /home/wheeltec/catkin_workspace/install/setup.bash --extend
export PYTHONPATH=/home/wheeltec/archiconda3/envs/wheeltec/bin:$PYTHONPATH
# added by Archiconda3 0.2.3 installer
# >>> conda init >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$(CONDA_REPORT_ERRORS=false '/home/wheeltec/archiconda3/bin/conda' sh$
if [ $? -eq 0 ]; then
    \eval "$__conda_setup"
else
    if [ -f "/home/wheeltec/archiconda3/etc/profile.d/conda.sh" ]; then
        . "/home/wheeltec/archiconda3/etc/profile.d/conda.sh"
        CONDA_CHANGEPS1=false conda activate base
    else
        \export PATH="/home/wheeltec/archiconda3/bin:$PATH"
    fi
fi
unset __conda_setup

alias sudo='sudo env PATH=$PATH'
# <<< conda init <<<
EOF
