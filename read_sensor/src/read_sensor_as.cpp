#include "read_sensor.h"
#include <actionlib/server/simple_action_server.h>
#include <read_sensor/ComputeBiasAction.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

/* Global Variables */
vector<string>   _DeviceName;
vector<Sensor_t> _SensorList;
uint8_t _SelectedSensorID;
serial::Serial *_SensorSerial;
uint16_t _TactileOffsetNumSamples;
bool b_startup = true;
vector<float> bias;

actionlib::SimpleActionServer<read_sensor::ComputeBiasAction>* compute_bias_as;


/*
    Compute bias routine
*/
bool computeBias(int num_samples_to_use, bool b_from_as = false){

    cout << BOLDYELLOW "Computing BIAS... "<< std::flush;

    //Initializate bias to zero
    vector<double> bias_;
    for( int i=0; i<25; i++ )
        bias_.push_back(0.0);

      vector<uint8_t> dataRaw;
    //Collect Samples for bias computation
    for( int i=0; i<num_samples_to_use; i++ ){
            _SensorSerial->write("a");
            _SensorSerial->read(dataRaw, _SensorList.at(_SelectedSensorID).NumSensingElements*2U);
        // cout << endl;
        for( int i=0; i<25; i++ )
            bias_[i] += (float)( dataRaw.at(i*2U)+dataRaw.at((i*2U)+1U)*255U ) * 3.3f /4096.0f;
        if(b_from_as){
            read_sensor::ComputeBiasFeedback msg_feedbk;
            msg_feedbk.samples_left = num_samples_to_use-i-1;
            compute_bias_as->publishFeedback(msg_feedbk);
        }
    }

    //Compute bias
    for( int i=0; i<25; i++ )
        bias[i] = bias_[i]/num_samples_to_use;

    cout << GREEN "DONE!" RESET << endl;

    return true;

}

/*
    Compute Bias Action CB
*/
void executeComputeBiasCB( const read_sensor::ComputeBiasGoalConstPtr &goal ){

    read_sensor::ComputeBiasResult remove_bias_result;

    if(goal->num_samples > 0){
        remove_bias_result.success = computeBias(goal->num_samples,true);

    } else {
        remove_bias_result.success = computeBias(TACTILE_OFFSET_NUM_SAMPLES,true);
    }

    if(remove_bias_result.success){
        remove_bias_result.msg = "DONE";
        remove_bias_result.bias.resize(25);
        remove_bias_result.bias = bias;
        compute_bias_as->setSucceeded(remove_bias_result);
    } else {
        remove_bias_result.msg = "ERROR - IS FINGERTIP ONLINE?";
        compute_bias_as->setAborted(remove_bias_result);
    }

}



/**
* @Brief This routine enumerates all Serial Ports of the PC.
* The output variables are:
* @li DeviceName: a vector with the device name.
* Returns the number of the found devices.
* IMPORTANT: Only the ttyUSB* and ttyACM* devices will be enumerated.
*/ 
uint8_t enumerate_ports(vector<string>& deviceName)
{
  char *nameCheck1;
  char *nameCheck2;
  const char *name1 = "ttyUSB";
  const char *name2 = "ttyACM";    
    
  vector<serial::PortInfo> devices_found = serial::list_ports();
  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while( iter != devices_found.end() )
  {
    serial::PortInfo device = *iter++;

    nameCheck1 = strstr((char*)device.port.c_str(), name1);
    nameCheck2 = strstr((char*)device.port.c_str(), name2);

    if(nameCheck1 || nameCheck2)
    {
      //printf( "(%s, %s, %s) %d\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str(), distance(devices_found.begin(), iter) );
      //printf( "(%s, %s, %s) ID: %d\n", devices_found.at(32).port.c_str() , device.description.c_str(), device.hardware_id.c_str(), distance(devices_found.begin(), DeviceID) );
      deviceName.push_back( device.port );
    }
  }

  /* Return the number of the found Serial devices */
  return deviceName.size();
}

/**
* @Brief This routine inizializes the sensors attached to the Serial Ports.
* It enumerates all the serial ports and if a compatible port is available checks
* the presence of a Tactile or Proximity sensor reading the sensor ID.
*/ 
void init_sensor(void)
{
  /* Private variables */
  uint8_t serialPortNum = 0U;
  
  /* Serial Port enumeration */
  serialPortNum = enumerate_ports(_DeviceName);
  
  /* No Serial port found. Exit with error. */
  if( !serialPortNum )
  {
    printf(BOLDRED "Error: " RESET "no Compatible Serial Ports found in the System. Please, make sure a ttyUSB* or a ttyACM* device is connected.\r\n");
    exit(-1);
  }
  /* Serial Port found. Print the ports. */
  else
  {
    printf(BOLDGREEN "Compatible Serial Ports found on the System: " RESET "%d\r\nChecking for compatible Sensor Devices...\r\n", serialPortNum);
    
    /* List the Device Ports */
    uint8_t sensorID = 0U;
    for(uint8_t k = 0U; k < serialPortNum; k++)
    {      
      /* Open Serial Port */
      /* port, baudrate, timeout in milliseconds */
      serial::Serial my_serial(_DeviceName.at(k), 1000000U, serial::Timeout::simpleTimeout(1000U));
      
      /* Check if a sensor is connected to the serial port */
      size_t bytes_wrote = my_serial.write("b");
      
      if ( bytes_wrote > 0U )
      {
        string result = my_serial.read(4U);
        
        if( 4U == result.length() )
        {
          /* Print header of sensor list */
          if(0U == sensorID)
          {
            printf("\r\nDevice List:\r\n");
            printf(BLUE "### PortID [#] - Port Name ## Sensor Name ###\r\n" RESET);
          }
          else
          {
            ;
          }

          /* Fill sensor information */
          Sensor_t currentSensor;
          currentSensor.SensorID = sensorID;
          currentSensor.SensorName = result;
          currentSensor.SerialPort = _DeviceName.at(k);
          
          _SensorList.push_back(currentSensor);
          
          /* Print Device Information */
          printf("### [%d] - %s ## ", currentSensor.SensorID, currentSensor.SerialPort.c_str());
          printf("%s ###\r\n", currentSensor.SensorName.c_str());
          
          /* Update sensor ID count */
          sensorID++;
        }
        else
        {
          ;
        }
      }
      else
      {
        ;
      }
      
      /* Close Serial Port */
      my_serial.close();
    }
    
    /* If at this point sensorID == 0U, then no sensor devices have been found */
    if(0U == sensorID)
    {       
      _SelectedSensorID = 255U;
    }
    else
    {
      _SelectedSensorID = 254U;
    }
  }
}

/**
* @Brief This routine inizializes the sensors attached to the Serial Port
* passed as input argument.
* It checks the presence of a Tactile or Proximity sensor reading the sensor ID.
*/ 
void init_sensor(string SerialPortName)
{
  uint8_t sensorID = 0U;
  
  try
  {
    /* Open Serial Port */
    /* port, baudrate, timeout in milliseconds */
    serial::Serial my_serial(SerialPortName, 1000000U, serial::Timeout::simpleTimeout(1000U));
    
    /* Check if a sensor is connected to the serial port */
    size_t bytes_wrote = my_serial.write("b");
    string result = my_serial.read(4U);
    
    if( 4U == result.length() )
    {
      /* Fill sensor information */
      Sensor_t currentSensor;
      currentSensor.SensorID = sensorID;
      currentSensor.SensorName = result;
      currentSensor.SerialPort = SerialPortName;
      
      _SensorList.push_back(currentSensor);
      
      /* Update sensor ID count */
      sensorID++;
    }
    else
    {
      ;
    }
    
    /* Close Serial Port */
    my_serial.close();
  }
  catch(std::exception& e)
  {
    printf(BOLDRED "Error: " RESET "cannot open the specified Serial Port. Port syntax: /dev/ttyUSB* or /dev/ttyACM*.\r\n");
    exit(-1);
  }
  
  /* If at this point sensorID == 0U, then no sensor devices have been found */
  if(0U == sensorID)
  {       
    _SelectedSensorID = 255U;
  }
  else
  {
    _SelectedSensorID = 0U;
  }
}

/**
* @Brief This routine allows to the user to select the sensor to use.
* If the input parameter 'SerialPortSpecified' is TRUE then the routine 
* simply configures the sensor data structure for the sensor attached to 
* the Serial Port specified by the user at the node instantiation. 
* If the input parameter 'SerialPortSpecified' is TRUE then the routine allows
* to choose a sensor between the sensors listed in _SensorList. 
* If _SelectedSensorID == 255U then no sensor is available.
*/ 
void select_sensor(vector<Sensor_t> sensorList, bool SerialPortSpecified)
{
  if( !SerialPortSpecified )
  {
    if( 255U == _SelectedSensorID )
    {
      printf(BOLDRED "\r\nNo sensors are available on the detected Serial Ports.\r\n" RESET);
      exit(-1);
    }
    else
    {
      while(1)
      {
        printf("\r\nPlease, select the Sensor to use by inserting the PortID #: ");
        scanf("%d", &_SelectedSensorID);
        
        if(_SelectedSensorID < _SensorList.size() && _SelectedSensorID >= 0)
        {
          break;
        }
        else
        {
          printf(RED "A wrong PortID has been selected.\r\n" RESET);
        }
      }
      
      printf(GREEN "Selected Sensor: %d\r\n" RESET, _SelectedSensorID);
      
      /* Open Serial Port */
      _SensorSerial = new serial::Serial(_SensorList.at(_SelectedSensorID).SerialPort, 1000000U, serial::Timeout::simpleTimeout(1000U));
      
      /* Set low latency option */
      char command[100] = {};
      strcat(command, "setserial ");
      strcat(command, _SensorList.at(_SelectedSensorID).SerialPort.c_str());
      strcat(command, " low_latency"); 
      system(command);
      
      /**
        * Read the number of sensing elements.
        * If the sensor is a proximity sensor, then ask for the number of
        * sensing elements, otherwise set it to 25.
        */
      if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P") )
      {
        size_t bytes_wrote = _SensorSerial->write("c");
        _SensorSerial->read(&_SensorList.at(_SelectedSensorID).NumSensingElements, 1U);
      }
      else if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "F") )
      {
        _SensorList.at(_SelectedSensorID).NumSensingElements = 25U;
      }
      else
      {
        ;
      }
      
      if( _SensorList.at(_SelectedSensorID).NumSensingElements > 0U )
      {
        printf(GREEN "Number of the Sensor Sensing Elements: %d\r\n" RESET, _SensorList.at(_SelectedSensorID).NumSensingElements);
      }
      else
      {
        printf(RED "No Sensing Elements available on the selected Sensor." RESET);
        exit(-1);
      }
    }
  }
  else if( SerialPortSpecified )
  {
    if( 255U == _SelectedSensorID )
    {
      printf(BOLDRED "\r\nNo sensors are available on the specified Serial Port.\r\n" RESET);
      exit(-1);
    }
    else
    {
      printf(BOLDGREEN "Serial Port %s is used for the current sensor.\r\n" RESET, _SensorList.at(_SelectedSensorID).SerialPort.c_str());
      
      /* Open Serial Port */
      _SensorSerial = new serial::Serial(_SensorList.at(_SelectedSensorID).SerialPort, 1000000U, serial::Timeout::simpleTimeout(1000U));
      
      /* Set low latency option */
      char command[100] = {};
      strcat(command, "setserial ");
      strcat(command, _SensorList.at(_SelectedSensorID).SerialPort.c_str());
      strcat(command, " low_latency"); 
      system(command);
      
      /**
        * Read the number of sensing elements.
        * If the sensor is a proximity sensor, then ask for the number of
        * sensing elements, otherwise set it to 25.
        */
      if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P") )
      {
        size_t bytes_wrote = _SensorSerial->write("c");
        _SensorSerial->read(&_SensorList.at(_SelectedSensorID).NumSensingElements, 1U);
      }
      else if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "F") )
      {
        _SensorList.at(_SelectedSensorID).NumSensingElements = 25U;
      }
      else
      {
        ;
      }
      
      if( _SensorList.at(_SelectedSensorID).NumSensingElements > 0U )
      {
        printf(GREEN "Number of the Sensor Sensing Elements: %d\r\n" RESET, _SensorList.at(_SelectedSensorID).NumSensingElements);
      }
      else
      {
        printf(RED "No Sensing Elements available on the selected Sensor." RESET);
        exit(-1);
      }
    }
  }
  else
  {
    ;
  }
}

int main(int argc, char* argv[])
{  
  /* Sensor Initialization */
  if (argc == 1U)
  {
    init_sensor();
    select_sensor(_SensorList, false);
  }
  else if (argc == 2U)
  {
    string userSerialPort = argv[1];
    init_sensor(userSerialPort);
    select_sensor(_SensorList, true);
  }
  else
  {
    ;
  }
  
  /* ROS Initialization */
  string nodeName = "SensorNode" + _SensorList.at(_SelectedSensorID).SerialPort + "_" + _SensorList.at(_SelectedSensorID).SensorName;
  replace( nodeName.begin(), nodeName.end(), '/', '_' );
  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;
  
  /* ROS publisher declaration */
  ros::Publisher sensor_data_pub_raw, sensor_data_pub;
  read_sensor::proximity_sensor_data proximity_sensor_data_msg;
  read_sensor::tactile_sensor_data tactile_sensor_data_raw_msg, tactile_sensor_data_msg;
  
  /* Sleep for a while */
  // printf("\r\nStarting sensor readings...\r\n");
  ros::Duration(2.0).sleep(); /* Sleep for 2 seconds */
  
  /* Set Publisher and Rate for P--- Sensors */
  uint16_t rate = 0U;
  if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P") )
  {
    string topicName_raw = "ProximityData" + _SensorList.at(_SelectedSensorID).SerialPort + "_" + _SensorList.at(_SelectedSensorID).SensorName;
    replace( topicName_raw.begin(), topicName_raw.end(), '/', '_' );
    sensor_data_pub_raw = n.advertise<read_sensor::proximity_sensor_data>(topicName_raw, 1);
    rate = 50U;
  }
  /* Set Publisher and Rate for F--- Sensors */
  else if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "F") )
  {
    string topicName_raw = "TactileData" + _SensorList.at(_SelectedSensorID).SerialPort + "_" + _SensorList.at(_SelectedSensorID).SensorName + "_raw";
    string topicName = "TactileData" + _SensorList.at(_SelectedSensorID).SerialPort + "_" + _SensorList.at(_SelectedSensorID).SensorName;
    replace( topicName_raw.begin(), topicName_raw.end(), '/', '_' );
    replace( topicName.begin(), topicName.end(), '/', '_' );
    sensor_data_pub_raw = n.advertise<read_sensor::tactile_sensor_data>(topicName_raw, 1);
    sensor_data_pub = n.advertise<read_sensor::tactile_sensor_data>(topicName, 1);
    
    /* Initialize the offset array for tactile sensor */
    _TactileOffsetNumSamples = 0U;
    for (uint16_t count = 0U; count < _SensorList.at(_SelectedSensorID).NumSensingElements; count++)
    {
      _SensorList.at(_SelectedSensorID).Offset.push_back( (float)0.0 );
    }
    
    rate = 500U;
  }
  else
  {
    ;
  }
  
  /* Temporary variable for RAW Sensor data */
  vector<uint8_t> dataRaw;
  
  /* Set Rate */
  ros::Rate loop_rate(rate);

  string action_compute_bias_str = _SensorList.at(_SelectedSensorID).SensorName+"/action_compute_bias";
  bias.resize(25);
  compute_bias_as = new actionlib::SimpleActionServer<read_sensor::ComputeBiasAction>(
                  n, 
                  action_compute_bias_str,
                  executeComputeBiasCB, 
                  false);

  compute_bias_as->start();
  
  /* Main loop */
  while (ros::ok())
  {
    dataRaw.clear();
    _SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.clear();
    
    /* Sensor readings for P--- Sensors */
    if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P") )
    {
      /* Request sensor data */
      size_t bytes_wrote = _SensorSerial->write("a");
      
      /* Read sensor data */ 
      proximity_sensor_data_msg.proximity_sensor_data.clear();
      _SensorSerial->read(dataRaw, _SensorList.at(_SelectedSensorID).NumSensingElements);
      
      /* Fill data buffer and ROS msg */
      for(int k = 0U; k < _SensorList.at(_SelectedSensorID).NumSensingElements; k++)
      {
        _SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.push_back(dataRaw.at(k));
        proximity_sensor_data_msg.proximity_sensor_data.push_back((float)dataRaw.at(k));
        
        /* Print sensor data values on the screen */
        printf("Data %d: %f mm\r\n", k+1, _SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.at(k));
      }
      
      /* Send data over ROS topic */
      proximity_sensor_data_msg.header.stamp = ros::Time::now();
      sensor_data_pub_raw.publish(proximity_sensor_data_msg);
    }
    /* Sensor readings for F--- Sensors */
    else if( strstr((char*)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "F") )
    {
      /* Request sensor data */
      size_t bytes_wrote = _SensorSerial->write("a");
      
      /* Read sensor data */ 
      tactile_sensor_data_raw_msg.tactile_sensor_data.clear();
      tactile_sensor_data_msg.tactile_sensor_data.clear();

      while (compute_bias_as->isActive()) {}
      
      _SensorSerial->read(dataRaw, _SensorList.at(_SelectedSensorID).NumSensingElements*2U);
      
      /* Fill data buffer and ROS msg */
      for(int k = 0U; k < _SensorList.at(_SelectedSensorID).NumSensingElements; k++)
      {
        _SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.push_back( (float)( dataRaw.at(k*2U)+dataRaw.at((k*2U)+1U)*255U ) * 3.3f /4096.0f );
        tactile_sensor_data_raw_msg.tactile_sensor_data.push_back(_SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.at(k));
      }
      if (b_startup) {
        computeBias(TACTILE_OFFSET_NUM_SAMPLES);
        b_startup = false;
      }


      for(int k = 0U; k < _SensorList.at(_SelectedSensorID).NumSensingElements; k++)
      {

        tactile_sensor_data_msg.tactile_sensor_data.push_back(_SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.at(k) - bias[k]);
        
        /* Print sensor data values on the screen */
        // printf("V%d: %f V\r\n", k+1, _SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.at(k));
      }
            
      /* Send data over ROS topic */
      tactile_sensor_data_raw_msg.header.stamp = ros::Time::now();
      tactile_sensor_data_msg.header.stamp = ros::Time::now();
      sensor_data_pub_raw.publish(tactile_sensor_data_raw_msg);
      sensor_data_pub.publish(tactile_sensor_data_msg);
    }
    else
    {
      ;
    }
    ros::spinOnce();
    /* Sleep until new iteration */
    loop_rate.sleep();
  }
  
  delete compute_bias_as;
  return 0;
}
