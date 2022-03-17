# Serial Port name for Sensor1
SerialPort1="/dev/ttyUSB1"

# Serial Port name for Sensor2
SerialPort2="/dev/ttyUSB3"

# ROS Init
gnome-terminal -e roscore
sleep 3

# ROS Node for Sensor1
gnome-terminal -x bash -c "rosrun read_sensor read_sensor_node $SerialPort1; exec bash"

# ROS Node for Sensor2
gnome-terminal -x bash -c "rosrun read_sensor read_sensor_node $SerialPort2; exec bash"
