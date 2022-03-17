To properly launch the 'read_sensor' ROS node, execute the following command:
1. Check if the current USER is in the 'dialout' group. Execute the 'groups' command in the terminal and verify that 'dialout' is listed.
2. If the current USER is not in the 'dialout' group, execute the following command in the terminal: sudo usermod -a -G dialout <user> where <user> is the name of the current user. Reboot the system.
3. If not installed, install 'setserial' tool with the following command: sudo apt install setserial
4. Execute 'roscore'
5. Execute 'rosrun read_sensor read_sensor_node'

4b. Alternatively you can execute the launch file 'read_sensors.sh' in 'read_serial/launch'. Opening it, it is possibile to configure the launch file with the right Sensor Serial Ports.
