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




ROS Update, remote server control

1. Install rosbridge
sudo apt-get install ros-melodic-rosbridge-server

echo $ROS_MASTER_URI
echo $ROS_HOSTNAME
ifconfig

export ROS_HOSTNAME=192.168.0.202

#to make it permanent, add the line to this file
source ~/.basrhc

roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
roslaunch rosbridge_server rosbridge_websocket.launch

gedit .bashrc