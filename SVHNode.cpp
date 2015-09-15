// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \author  Georg Heppner
 * \date    2014-09-23
 *
 */
//----------------------------------------------------------------------
// ROS includes.
#include <ros/ros.h>
#include <string>

// Custom includes
#include "SVHNode.h"
#include <icl_core/EnumHelper.h>
#include <math.h>
#include <time.h>
#include <TooN/TooN.h>
#include <TooN/GR_SVD.h>
#include <TooN/Lapack_Cholesky.h>
#include <TooN/lapack.h>
#include <TooN/SVD.h>
//#include <TooN/LU.h>
#include <iostream>
#include <fstream>
//#include <stdio.h>
//#include <TooN/QR_Lapack.h>
#include <TooN/SymEigen.h>

using namespace std;
using namespace TooN;

/*--------------------------------------------------------------------
 * Callback functions
 *------------------------------------------------------------------*/

SVHNode::SVHNode(const ros::NodeHandle & nh)
{

  //==========
  // Params
  //==========

  bool autostart,use_internal_logging;
  int reset_timeout;
  std::vector<bool> disable_flags(driver_svh::eSVH_DIMENSION, false);
  // Config that contains the log stream configuration without the file names
  std::string logging_config_file;
  // File to store the debug and log files in
  //std::string log_debug_file,log_trace_file;



  try
  {
    nh.param<bool>("autostart",autostart,false);
    nh.param<bool>("use_internal_logging",use_internal_logging,false);
    nh.param<std::string>("serial_device",serial_device_name_,"/dev/ttyUSB0");
    // Note : Wrong values (like numerics) in the launch file will lead to a "true" value here
    nh.getParam("disable_flags",disable_flags);
    nh.param<int>("reset_timeout",reset_timeout,5);
    nh.getParam("logging_config",logging_config_file);
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error!");
  }

  // Initialize the icl_library logging framework ( THIS NEEDS TO BE DONE BEFORE ANY LIB OBJECT IS CREATED)
  if (use_internal_logging)
  {
    // Fake an input to the logging call to tell it where to look for the logging config

    // Strdup to create non const chars as it is required by the initialize function.
    // not really beatiful but it works.
    char * argv[]= {
      strdup("Logging"),
      strdup("-c"),
      strdup(logging_config_file.c_str())
    };
    int argc = 3; // number of elements above

    // In case the file is not present (event though the parameter is) the logging will just put out a
    // warning so we dont need to check it further. However the log level will only be Info (out of the available Error, Warning, Info, Debug, Trace)
    // in that case also the log files will be disregarded
    if (icl_core::logging::initialize(argc,argv))
      ROS_INFO("Internal logging was activated, output will be written as configured in logging.xml (default to ~/.ros/log)");
    else
      ROS_WARN("Internal logging was enabled but config file could not be read. Please make sure the provided path to the config file is correct.");
  }
  else
  {
    icl_core::logging::initialize();
  }

  for (size_t i = 0; i < 9; ++i)
  {
    if(disable_flags[i])
    {
      ROS_WARN("svh_controller disabling channel nr %i", i);
    }
  }

  // Init the actual driver hook (after logging initialize)
  fm_.reset(new driver_svh::SVHFingerManager(disable_flags,reset_timeout));

  // Rosparam can only fill plain vectors so we will have to go through them
  std::vector< std::vector<float> > position_settings(driver_svh::eSVH_DIMENSION);
  std::vector<bool> postion_settings_given(driver_svh::eSVH_DIMENSION,false);

  std::vector< std::vector<float> > current_settings(driver_svh::eSVH_DIMENSION);
  std::vector<bool> current_settings_given(driver_svh::eSVH_DIMENSION,false);

  std::vector< std::vector<float> > home_settings(driver_svh::eSVH_DIMENSION);
  std::vector<bool> home_settings_given(driver_svh::eSVH_DIMENSION,false);


  // get the the indidividual finger params
  // We will read out all of them, so that in case we fail half way we do not set anything
  try
  {
    postion_settings_given[driver_svh::eSVH_THUMB_FLEXION] = nh.getParam("THUMB_FLEXION/position_controller",position_settings[driver_svh::eSVH_THUMB_FLEXION]);
    current_settings_given[driver_svh::eSVH_THUMB_FLEXION] = nh.getParam("THUMB_FLEXION/current_controller",current_settings[driver_svh::eSVH_THUMB_FLEXION]);
    home_settings_given[driver_svh::eSVH_THUMB_FLEXION]    = nh.getParam("THUMB_FLEXION/home_settings",home_settings[driver_svh::eSVH_THUMB_FLEXION]);

    postion_settings_given[driver_svh::eSVH_THUMB_OPPOSITION] = nh.getParam("THUMB_OPPOSITION/position_controller",position_settings[driver_svh::eSVH_THUMB_OPPOSITION]);
    current_settings_given[driver_svh::eSVH_THUMB_OPPOSITION] = nh.getParam("THUMB_OPPOSITION/current_controller",current_settings[driver_svh::eSVH_THUMB_OPPOSITION]);
    home_settings_given[driver_svh::eSVH_THUMB_OPPOSITION]    = nh.getParam("THUMB_OPPOSITION/home_settings",home_settings[driver_svh::eSVH_THUMB_OPPOSITION]);

    postion_settings_given[driver_svh::eSVH_INDEX_FINGER_DISTAL] = nh.getParam("INDEX_FINGER_DISTAL/position_controller",position_settings[driver_svh::eSVH_INDEX_FINGER_DISTAL]);
    current_settings_given[driver_svh::eSVH_INDEX_FINGER_DISTAL] = nh.getParam("INDEX_FINGER_DISTAL/current_controller",current_settings[driver_svh::eSVH_INDEX_FINGER_DISTAL]);
    home_settings_given[driver_svh::eSVH_INDEX_FINGER_DISTAL]    = nh.getParam("INDEX_FINGER_DISTAL/home_settings",home_settings[driver_svh::eSVH_INDEX_FINGER_DISTAL]);

    postion_settings_given[driver_svh::eSVH_INDEX_FINGER_PROXIMAL] = nh.getParam("INDEX_FINGER_PROXIMAL/position_controller",position_settings[driver_svh::eSVH_INDEX_FINGER_PROXIMAL]);
    current_settings_given[driver_svh::eSVH_INDEX_FINGER_PROXIMAL] = nh.getParam("INDEX_FINGER_PROXIMAL/current_controller",current_settings[driver_svh::eSVH_INDEX_FINGER_PROXIMAL]);
    home_settings_given[driver_svh::eSVH_INDEX_FINGER_PROXIMAL]    = nh.getParam("INDEX_FINGER_PROXIMAL/home_settings",home_settings[driver_svh::eSVH_INDEX_FINGER_PROXIMAL]);

    postion_settings_given[driver_svh::eSVH_MIDDLE_FINGER_DISTAL] = nh.getParam("MIDDLE_FINGER_DISTAL/position_controller",position_settings[driver_svh::eSVH_MIDDLE_FINGER_DISTAL]);
    current_settings_given[driver_svh::eSVH_MIDDLE_FINGER_DISTAL] = nh.getParam("MIDDLE_FINGER_DISTAL/current_controller",current_settings[driver_svh::eSVH_MIDDLE_FINGER_DISTAL]);
    home_settings_given[driver_svh::eSVH_MIDDLE_FINGER_DISTAL]    = nh.getParam("MIDDLE_FINGER_DISTAL/home_settings",home_settings[driver_svh::eSVH_MIDDLE_FINGER_DISTAL]);

    postion_settings_given[driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL] = nh.getParam("MIDDLE_FINGER_PROXIMAL/position_controller",position_settings[driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL]);
    current_settings_given[driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL] = nh.getParam("MIDDLE_FINGER_PROXIMAL/current_controller",current_settings[driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL]);
    home_settings_given[driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL]    = nh.getParam("MIDDLE_FINGER_PROXIMAL/home_settings",home_settings[driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL]);

    postion_settings_given[driver_svh::eSVH_RING_FINGER] = nh.getParam("RING_FINGER/position_controller",position_settings[driver_svh::eSVH_RING_FINGER]);
    current_settings_given[driver_svh::eSVH_RING_FINGER] = nh.getParam("RING_FINGER/current_controller",current_settings[driver_svh::eSVH_RING_FINGER]);
    home_settings_given[driver_svh::eSVH_RING_FINGER]    = nh.getParam("RING_FINGER/home_settings",home_settings[driver_svh::eSVH_RING_FINGER]);

    postion_settings_given[driver_svh::eSVH_PINKY] = nh.getParam("PINKY/position_controller",position_settings[driver_svh::eSVH_PINKY]);
    current_settings_given[driver_svh::eSVH_PINKY] = nh.getParam("PINKY/current_controller",current_settings[driver_svh::eSVH_PINKY]);
    home_settings_given[driver_svh::eSVH_PINKY]    = nh.getParam("PINKY/home_settings",home_settings[driver_svh::eSVH_PINKY]);

    postion_settings_given[driver_svh::eSVH_FINGER_SPREAD] = nh.getParam("FINGER_SPREAD/position_controller",position_settings[driver_svh::eSVH_FINGER_SPREAD]);
    current_settings_given[driver_svh::eSVH_FINGER_SPREAD] = nh.getParam("FINGER_SPREAD/current_controller",current_settings[driver_svh::eSVH_FINGER_SPREAD]);
    home_settings_given[driver_svh::eSVH_FINGER_SPREAD]    = nh.getParam("FINGER_SPREAD/home_settings",home_settings[driver_svh::eSVH_FINGER_SPREAD]);

    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      // Only update the values in case actually have some. Otherwise the driver will use internal defaults. Overwriting them with zeros would be counter productive
      if (current_settings_given[channel])
      {
        fm_->setCurrentSettings(static_cast<driver_svh::SVHChannel>(channel),driver_svh::SVHCurrentSettings(current_settings[channel]));
      }

      if (postion_settings_given[channel])
      {
        fm_->setPositionSettings(static_cast<driver_svh::SVHChannel>(channel),driver_svh::SVHPositionSettings(position_settings[channel]));
      }
      if (home_settings_given[channel])
      {
        fm_->setHomeSettings(static_cast<driver_svh::SVHChannel>(channel),driver_svh::SVHHomeSettings(home_settings[channel]));
      }
    }
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error! While reading the controller settings. Will use default settings");
  }

  // prepare the channel position message for later sending
  channel_pos_.name.resize(driver_svh::eSVH_DIMENSION);
  channel_pos_.position.resize(driver_svh::eSVH_DIMENSION, 0.0);
  channel_pos_.effort.resize(driver_svh::eSVH_DIMENSION, 0.0); // HO AGGIUNTO CAMPO EFFORT PER LEGGERE ISTANTE PER ISTANTE IL VALORE DI CORRENTE
  
  channel_cost_.name.resize(driver_svh::eSVH_DIMENSION);
  channel_cost_.position.resize(driver_svh::eSVH_DIMENSION, 0.0);
  channel_cost_.effort.resize(driver_svh::eSVH_DIMENSION, 0.0);
  channel_cost_.velocity.resize(3, 0.0); // In questo modo prendo anche il vettore dei pesi
  
  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    channel_pos_.name[channel] = driver_svh::SVHController::m_channel_description[channel];
    channel_cost_.name[channel] = driver_svh::SVHController::m_channel_description[channel];
  }

  // Connect and start the reset so that the hand is ready for use
  if (autostart && fm_->connect(serial_device_name_))
  {
    fm_->resetChannel(driver_svh::eSVH_ALL);
    ROS_INFO("Driver was autostarted! Input can now be sent. Have a safe and productive day!");
  }
  else
  {
    ROS_INFO("SVH Driver Ready, you will need to connect and reset the fingers before you can use the hand.");
  }

}

SVHNode::~SVHNode()
{
  // Tell the driver to close connections
  fm_->disconnect();
}

// Callback function for changing parameters dynamically
void SVHNode::dynamic_reconfigure_callback(svh_controller::svhConfig &config, uint32_t level)
{
  serial_device_name_ = config.serial_device;

  fm_->setResetSpeed(config.finger_reset_speed);
  fm_->setResetTimeout(config.reset_timeout);
}

// Callback function for connecting to SCHUNK five finger hand
void SVHNode::connectCallback(const std_msgs::Empty&)
{
  if (fm_->isConnected())
  {
    fm_->disconnect();
  }

  if (!fm_->connect(serial_device_name_))
  {
    ROS_ERROR("Could not connect to SCHUNK five finger hand with serial device %s", serial_device_name_.c_str());
  }
}

// Callback function to reset/home channels of SCHUNK five finger hand
void SVHNode::resetChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  // convert int8 channel into SVHChannel enum
  driver_svh::SVHChannel svh_channel = static_cast<driver_svh::SVHChannel>(channel->data);

  if (fm_->resetChannel(svh_channel))
  {
    ROS_INFO("Channel %i successfully homed!", svh_channel);
  }
  else
  {
    ROS_ERROR("Could not reset channel %i !", svh_channel);
  }
}

// Callback function to enable channels of SCHUNK five finger hand
void SVHNode::enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  fm_->enableChannel(static_cast<driver_svh::SVHChannel>(channel->data));
}

// Callback function for Matlab_Node
void SVHNode::chatterCallback(const std_msgs::Int8ConstPtr& msg)
{
  ROS_INFO("Indice: %d ", msg->data);
  //rate.sleep();
}

// Callback function for setting channel target positions to SCHUNK five finger hand
void SVHNode::jointStateCallback(const sensor_msgs::JointStateConstPtr& input )
{
  // vector to read target positions from joint states
  std::vector<double> target_positions(driver_svh::eSVH_DIMENSION, 0.0);
  // bool vector storing true, if new target position read
  std::vector<bool> pos_read(driver_svh::eSVH_DIMENSION, false);
  // target positions read counter
  uint8_t pos_read_counter = 0;

  size_t index = 0;
  std::vector<std::string>::const_iterator joint_name;
  for (joint_name = input->name.begin(); joint_name != input->name.end(); ++joint_name,++index)
  {
    int32_t channel = 0;
    if (icl_core::string2Enum((*joint_name), channel, driver_svh::SVHController::m_channel_description))
    {
      if (input->position.size() > index)
      {
        target_positions[channel] = input->position[index];
        pos_read[channel] = true;
        pos_read_counter++;
      }
      else
      {
        ROS_WARN("Vector of input joint state is too small! Cannot acces elemnt nr %i", index);
      }
    }
    else
    {
      //ROS_WARN("Could not map joint name %s to channel!", (*joint_name).c_str());
    }
  }

  // send target values at once
  if (pos_read_counter == driver_svh::eSVH_DIMENSION)
  {
    if (!fm_->setAllTargetPositions(target_positions))
    {
      ROS_WARN_ONCE("Set target position command rejected!");
    }
  }
  // not all positions has been set: send only the available positions
  else
  {
    for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; ++i)
    {
      if (pos_read[i])
      {
        fm_->setTargetPosition(static_cast<driver_svh::SVHChannel>(i), target_positions[i], 0.0);
      }
    }
  }
}


sensor_msgs::JointState SVHNode::getCurrentPositions() //MODIFICATO PER SALVARE ANCHE LA CORRENTE E NON SOLO LA POSIZIONE
{
  if (fm_->isConnected())
  {
    // Get positions in rad
    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      double cur_pos = 0.0;
      double cur_cur = 0.0;
      if (fm_->isHomed(static_cast<driver_svh::SVHChannel>(channel)))
      {
        fm_->getPosition(static_cast<driver_svh::SVHChannel>(channel), cur_pos);
        // Read out currents if you want to
        fm_->getCurrent(static_cast<driver_svh::SVHChannel>(channel),cur_cur);
      }
      channel_pos_.position[channel] = cur_pos;
      channel_pos_.effort[channel]= cur_cur;

      channel_cost_.position[channel] = cur_pos;
      channel_cost_.effort[channel]= cur_cur;
    }
  }

  channel_pos_.header.stamp = ros::Time::now();

  channel_cost_.header.stamp = ros::Time::now();      
  return  channel_pos_;
}



/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  //==========
  // ROS
  //==========

  // Set up ROS.
  ros::init(argc, argv, "svh_controller");
  // Private NH for general params
  ros::NodeHandle nh("~");


  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(100);  //QUA MODIFICHI LA FREQUENZA DI CAMPIONAMENTO

  //==========
  // Logic
  //==========
  // Node object holding all the relevant functions
  SVHNode svh_node(nh);

  //==========
  // Dynamic Reconfigure
  //==========

  dynamic_reconfigure::Server<svh_controller::svhConfig> server;
  dynamic_reconfigure::Server<svh_controller::svhConfig>::CallbackType f;

  f = boost::bind(&SVHNode::dynamic_reconfigure_callback,&svh_node, _1, _2);
  server.setCallback(f);

  //==========
  // Callbacks
  //==========

  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = nh.subscribe("connect", 1, &SVHNode::connectCallback,&svh_node);
  // Subscribe chatter topic
  ros::Subscriber chatter_sub = nh.subscribe("/chatter", 1, &SVHNode::chatterCallback, &svh_node);
  // Subscribe reset channel topic (Int8)
  ros::Subscriber reset_sub = nh.subscribe("reset_channel", 1, &SVHNode::resetChannelCallback,&svh_node);
  // Subscribe enable channel topic (Int8)
  ros::Subscriber enable_sub = nh.subscribe("enable_channel", 1, &SVHNode::enableChannelCallback, &svh_node);
  // Subscribe joint state topic
  ros::Subscriber channel_target_sub = nh.subscribe<sensor_msgs::JointState>("channel_targets", 1, &SVHNode::jointStateCallback,&svh_node );
  // Publish current channel positions
  ros::Publisher channel_pos_pub = nh.advertise<sensor_msgs::JointState>("channel_feedback", 1);//Controllo nel sottospazio delle sinergie
  ros::Publisher channel_ced_pub = nh.advertise<sensor_msgs::JointState>("channel_targets", 1);//Aggiunti pe ril controllo in corrente (inserimento soglie)
  // Topic cost function
  ros::Publisher channel_cost_pub = nh.advertise<sensor_msgs::JointState>("syn", 1);

  //==========
  // Messaging
  //==========

  // joint state message template
    sensor_msgs::JointState channel_pos;
    channel_pos.name.resize(9);
    channel_pos.position.resize(9, 0.0);
    channel_pos.effort.resize(9, 0.0);

    sensor_msgs::JointState channel_cost;
    channel_cost.name.resize(9);
    channel_cost.position.resize(9, 0.0);
    channel_cost.effort.resize(9, 0.0);
    channel_cost.velocity.resize(3, 0.0);   

    // Pre Fill the Names of the Finger
    channel_pos.name[0] = "Thumb_Flexion";
    channel_pos.name[1] = "Thumb_Opposition";
    channel_pos.name[2] = "Index_Finger_Distal";
    channel_pos.name[3] = "Index_Finger_Proximal";
    channel_pos.name[4] = "Middle_Finger_Distal";
    channel_pos.name[5] = "Middle_Finger_Proximal";
    channel_pos.name[6] = "Ring_Finger";
    channel_pos.name[7] = "Pinky";
    channel_pos.name[8] = "Finger_Spread";

    channel_cost.name[0] = "Thumb_Flexion";
    channel_cost.name[1] = "Thumb_Opposition";
    channel_cost.name[2] = "Index_Finger_Distal";
    channel_cost.name[3] = "Index_Finger_Proximal";
    channel_cost.name[4] = "Middle_Finger_Distal";
    channel_cost.name[5] = "Middle_Finger_Proximal";
    channel_cost.name[6] = "Ring_Finger";
    channel_cost.name[7] = "Pinky";
    channel_cost.name[8] = "Finger_Spread";

    #ifndef PI
    #define PI	3.1415926535897932384626433832795
    #endif
    const float Rad = 0.017453292519943295769236907684886;
       
    #define NUMBER_OF_JOINTS	20
    #define N_MOTORS 9
        
    //ad ogni istante di campionamento la posizione varia di 0.02 radianti(circa 1 grado
    Matrix <1,9> m;   
    Vector<9> d;
    for(size_t channel = 0; channel < 9; ++channel)
     {
         d[channel]=0.02;
     }

         for(size_t channel = 0; channel < 9; ++channel)
          {

             channel_pos.position[channel]=svh_node.getCurrentPositions().position[channel];
             if(channel==0)
             {
                           m[0][1]=(svh_node.getCurrentPositions().position[channel]*180)/PI;
             }
             else{
                  if(channel==1)
                  {
                           m[0][0]=(svh_node.getCurrentPositions().position[channel]*180)/PI;
                  }                                
                  else
                  {
                           m[0][channel]=(svh_node.getCurrentPositions().position[channel]*180)/PI;}
                  }
             }

          int count = 1;
          float dt = 0.01;
          float delta_t = 5;
          Vector<9> fine_corsa_max;
          fine_corsa_max = Data(0.93, 0.95, 1.30, 0.76, 1.29, 0.76, 0.94, 0.94, 0.55);        
          Matrix<15,1> dx_q, dx_q0;
          Vector<15> x_q;
        
          float x_pollice_plus;
          float y_pollice_plus;
          float z_pollice_plus;

          float x_indice_plus;
          float y_indice_plus;
          float z_indice_plus;

          float x_medio_plus;
          float y_medio_plus;
          float z_medio_plus;

          float x_anulare_plus;
          float y_anulare_plus;
          float z_anulare_plus;

          float x_mignolo_plus;
          float y_mignolo_plus;
          float z_mignolo_plus;


          Vector<9> mean_value;
          mean_value=Data(0.2109, 0.4064,    0.3020,    0.5998,    0.3338,    0.6404,    0.4899,    0.4960,    0.5384);
          
          //Il riferimento parte sempre dalla configurazione home
          float ref_pollice_x_0=98.7532;
          float ref_pollice_y_0=94.3440;                                                         
          float ref_pollice_z_0=45.5394;

          float ref_indice_x_0=39.1369; 
          float ref_indice_y_0=171.8648;
          float ref_indice_z_0=64.1132;

          float ref_medio_x_0=0.0;
          float ref_medio_y_0=178.6513;
          float ref_medio_z_0=57.8987;

          float ref_anulare_x_0=-34.6614; 
          float ref_anulare_y_0=174.1616; 
          float ref_anulare_z_0=53.1063;

          float ref_mignolo_x_0=-66.7618;  
          float ref_mignolo_y_0=149.2061;
          float ref_mignolo_z_0=43.7018;

          //configurazione degli angoli di giunto nella posizione home (in radianti e nell'ordine in cui devono essere pubblicati)
          Vector<9> q_home;
          q_home=Data(0.0523115, 0.0941094, 0.152329, 0.10236, 0.152861, 0.10214, 0.110544, 0.111568, 0.28469);

          float ref_pollice_x;
          float ref_pollice_y;
          float ref_pollice_z;

          float ref_indice_x;
          float ref_indice_y;
          float ref_indice_z;

          float ref_medio_x;
          float ref_medio_y;
          float ref_medio_z;

          float ref_anulare_x;
          float ref_anulare_y;
          float ref_anulare_z;

          float ref_mignolo_x;
          float ref_mignolo_y;
          float ref_mignolo_z;

          float x_mis_pollice;
          float y_mis_pollice;
          float z_mis_pollice;

          float x_mis_indice;
          float y_mis_indice;
          float z_mis_indice;

          float x_mis_medio;
          float y_mis_medio;
          float z_mis_medio;

          float x_mis_anulare;
          float y_mis_anulare;
          float z_mis_anulare;

          float x_mis_mignolo;
          float y_mis_mignolo;
          float z_mis_mignolo;

           //Vettore obj_o è il vettore centroide oggetto
          Vector <3> obj_o;


          int dita_contatto;
          Matrix <3,1> dq_k_precedente, dq_k, sigma;
          Matrix <15,1> errore;
          
          Vector<15> err; //DA INIZIALIZZARE A 0
          float q1, q2, q3, q4, q5, q6a, q6b, q7, q8a, q8b, q9a, q9b, q10, q11, q12, q13, q14, q15, q16, q17;

          Matrix<4,4> TP0=Zeros, TI0=Zeros, TM0=Zeros, TA0=Zeros, TMI0=Zeros;

              TP0=Data(1, 0,                  0,          16.90,
                       0, sin(Rad*15),    cos(Rad*15), 45.96*cos(Rad*15),
                       0, -cos(Rad*15),   sin(Rad*15), 20.26+45.96*sin(Rad*15),
                       0,     0,               0,                 1   );


              TI0=Data(0, -1, 0, 25,
                       1, 0, 0, 110,
                       0, 0, 1, 6,
                       0, 0, 0, 1);


             TM0=Data ( 0, 0, 1, 0,
                       1, 0, 0, 110,
                       0, 1, 0, -6,
                       0, 0, 0, 1);


              TA0=Data ( 1, 0, 0, -18.40,
                       0, 0, -1, 44.3940,
                       0, 1, 0, 0,
                       0, 0, 0, 1);



              TMI0=Data ( 1, 0, 0, -18.40,
                       0, 0, -1, 44.3940,
                       0, 1, 0, 0,
                       0, 0, 0, 1);

              Matrix<4,4> DH_P=Zeros, DH_I=Zeros;
                  //Tabella di DH (a,alpha,d,theta) dal secondo giunto del polso fino al POLLICE
                  DH_P =Data (0, 	90,       0,   0,
                              48.50,   0,       0,   0,
                              30,      0,       0,   0,
                              20,      0,       0,   0);
                  //Tabella di DH (a,alpha,d, theta) dal secondo giunto del polso fino al INDICE
                  DH_I =Data (0,          90,         0,    0,
                              48.04,      0,          0,      0,
                              26,         0,          0,      0,
                              15,         0,          0,       0);


                  Matrix<3,4> DH_M=Zeros;
                  //Tabella di DH (a,alpha,d,theta) dal secondo giunto del polso fino al MEDIO
                  DH_M =Data (50.04,       0,          0,   0,
                              32,          0,          0,  0,
                              15,          0,          0,   0);

                  Matrix<5,4> DH_A=Zeros, DH_MI=Zeros;
                  //Tabella di DH (a,alpha,d,theta) dal secondo giu.95 nto del polso fino all' ANULARE
                  DH_A =Data (0,   	   90,        -103+44.3940,       0,
                              0,	   -90,          6,       	  0,
                              50.04,      0,          0,                0,
                              32,         0,          0,                0,
                              15,         0,          0,                0);

                  //Tabella di DH (a,alpha,d,theta) dal secondo giunto del polso fino al MIGNOLO
                  DH_MI = Data(
                      -21.50,	    90,          -103+44.3940+9.5,       	0,
                      0,	   -90,          6,       			0,
                      44.54,       0,          0,       			0,
                      22,          0,          0,       			0,
                      15,          0,          0,       			0);

                  //Inizializzazione delle matrici di trasformazione omogenee utilizzate nei cicli for
                  Matrix<4,4>  DH_TP1=Zeros, DH_TP2=Zeros,DH_TP3=Zeros, DH_TP4=Zeros,
                               DH_TI1=Zeros, DH_TI2=Zeros,DH_TI3=Zeros, DH_TI4=Zeros,
                               DH_MI1=Zeros, DH_MI2=Zeros,DH_MI3=Zeros,
                               DH_TA1=Zeros, DH_TA2=Zeros,DH_TA3=Zeros, DH_TA4=Zeros, DH_TA5=Zeros,
                               DH_TMI1=Zeros, DH_TMI2=Zeros,DH_TMI3=Zeros, DH_TMI4=Zeros, DH_TMI5=Zeros;


                  double sq1,sq2,sq3,sq4,sq5,sq6a,sq6b,sq7,sq8a,sq8b,sq9a,sq9b,sq10,sq11,sq12,sq13,sq14,sq15,sq16,sq17,
                         cq1,cq2,cq3,cq4,cq5,cq6a,cq6b,cq7,cq8a,cq8b,cq9a,cq9b,cq10,cq11,cq12,cq13,cq14,cq15,cq16,cq17;
                  //Cinematica diretta
                  Matrix <4,4> H_p0, H_p1, H_p2, H_p3, H_pe, H_i0, H_i1, H_i2, H_i3, H_ie, H_m0, H_m1, H_m2, H_me, H_a0, H_a1, H_a2, H_a3, H_a4, H_ae, H_mi0, H_mi1, H_mi2, H_mi3, H_mi4, H_mie;

                  Matrix <3,2> Jp=Zeros;
                  Matrix <3,3> Ji=Zeros;
                  Matrix <3,3> Jm=Zeros;
                  Matrix <3,3> Ja=Zeros;
                  Matrix <3,3> Jmi=Zeros;

                  Vector <3> a, b_temp1, b_temp2, b;
                  Matrix <15,9> J=Zeros;
                  Matrix <3,1> dq;
                  Matrix<15,3> Jac=Zeros;
                  Matrix <1,9> q_ros, m_rad_ros, m_rad_riordinato;
                  Vector<3> k_pollice, k_indice, k_medio, k_anulare, k_mignolo;
                  Vector <3> xp, xi, xme, xa, xmi;
                  Vector <15> riferimento;

          //Matrice E delle sinergie
          Matrix<9,3> E=Zeros;

          E=Data(0.0808,   -0.2394,   0.9166,
                -0.0292,   0.1750,   0.2605,
                 0.1957,    0.0506,    0.0923,
                 0.1057,   -0.1799,   -0.1062,
                 0.3267,    0.1378,    0.1411,
                 0.1264,   -0.1800,   -0.0965,
                 0.6222,   -0.1734,   -0.1864,
                 0.5809,   -0.2508,   -0.0305,
                 0.3095,    0.8560,    0.0856);

          //Riferimento finale determinato per ciascuna delle 36 configurazioni di riferimento con applicazione PCA

          float ref_pollice_x_max;
          float ref_pollice_y_max;
          float ref_pollice_z_max;

          float ref_indice_x_max;
          float ref_indice_y_max;
          float ref_indice_z_max;
          float ref_medio_x_max;
          float ref_medio_y_max;
          float ref_medio_z_max;

          float ref_anulare_x_max;
          float ref_anulare_y_max;
          float ref_anulare_z_max;

          float ref_mignolo_x_max;
          float ref_mignolo_y_max;
          float ref_mignolo_z_max;



          Vector<9> q_pca;
          //Vector<9> q_syn_iniziale, q_syn_finale;
          //Scegliendo la presa varia il riferimento e le dita in contatto (per il calcolo del centroide)!!!
          int presa=27; //da 1 a 36 set di riferimento - presa=0 mean_value
          bool correzione=true; // se vuoi correggere il riferimento con il vettore che tiene conto del centroide
          bool standard=false; // se sta a true non avvia il nostro sw ma aspetta che tu da un altro terminale gli dia un riferimento in gradianti di posizione (come suggerito dal file di testo quick commands)
          //bool syn=false;

          if(presa==0)
             {

           ref_pollice_x_max=70.3104;
           ref_pollice_y_max=108.4663;
           ref_pollice_z_max=72.0678;

           ref_indice_x_max=24.9784 ;
           ref_indice_y_max=109.5117 ;
           ref_indice_z_max= 75.2226  ;
           ref_medio_x_max=0;
           ref_medio_y_max=105.2838;
           ref_medio_z_max= 71.2092;

           ref_anulare_x_max=1.8130;
           ref_anulare_y_max=84.5230   ;
           ref_anulare_z_max=46.9900;

           ref_mignolo_x_max=-21.5204;
           ref_mignolo_y_max= 81.7312;
           ref_mignolo_z_max=46.1873; 
           dita_contatto=5;

           //q_pca=Data(0.1528,    0.6350,    0.2076,    0.6296,    0.1600,    0.6987,    0.4735,    0.4798,    0.5725);

          // Mean Value        
           q_pca=Data(0.2109,    0.4064,    0.3020,    0.5998,    0.3338,    0.6404,    0.4899,    0.4960,    0.5384);

          }
         if(presa==1)
             {

           ref_pollice_x_max=63.4245;
           ref_pollice_y_max=95.2225;
           ref_pollice_z_max=96.7500;

           ref_indice_x_max=30.3239 ;
           ref_indice_y_max=127.9501;
           ref_indice_z_max= 89.7895 ;
           ref_medio_x_max=0;
           ref_medio_y_max=126.2724;
           ref_medio_z_max= 86.7416  ;

           ref_anulare_x_max=30.8561;
           ref_anulare_y_max=118.7802  ;
           ref_anulare_z_max=52.9805;

           ref_mignolo_x_max=4.0257;
           ref_mignolo_y_max=104.9511;
           ref_mignolo_z_max=61.0159;
          dita_contatto=5;


          /*q_pca[0]=0.1303;
          q_pca[1]=0.8137;
          q_pca[2]=0.0854;
          q_pca[3]=0.7051;
          q_pca[4]=0.0476;
          q_pca[5]=0.7587;
          q_pca[6]=0.3995;
          q_pca[7]=0.4069;
          q_pca[8]=0.5767;*/

          q_pca[0]=0.1361;
          q_pca[1]=0.5004;
          q_pca[2]=0.2145;
          q_pca[3]=0.6868;
          q_pca[4]=0.1747;
          q_pca[5]=0.7117;
          q_pca[6]=0.4419;
          q_pca[7]=0.4506;
          q_pca[8]=0.5767;

          }

          if(presa==2)
             {

           ref_pollice_x_max=63.0919;
           ref_pollice_y_max=102.4597;
           ref_pollice_z_max=88.3202;

           ref_indice_x_max=24.7271;
           ref_indice_y_max=108.9657;
           ref_indice_z_max= 86.4258 ;
           ref_medio_x_max=0;
           ref_medio_y_max=106.2155;
           ref_medio_z_max= 80.9006  ;

           ref_anulare_x_max=19.3942;


           ref_anulare_y_max=86.6410 ;
           ref_anulare_z_max=37.8416;

           ref_mignolo_x_max=-3.0452;
           ref_mignolo_y_max=83.8411 ;
           ref_mignolo_z_max=42.8367;
          dita_contatto=5;


          q_pca[0]=0.1809;
          q_pca[1]=0.7040;
          q_pca[2]=0.3175;
          q_pca[3]=0.7985;
          q_pca[4]=0.3486;
          q_pca[5]=0.7985;
          q_pca[6]=0.6993;
          q_pca[7]=0.6902;
          q_pca[8]=0.5159;

          }

          if(presa==3)
             {
           ref_pollice_x_max=66.0081;
           ref_pollice_y_max=104.6869 ;
           ref_pollice_z_max=82.8047;

           ref_indice_x_max=26.3761  ;
           ref_indice_y_max=118.1059;
           ref_indice_z_max=  87.6009;
           ref_medio_x_max=0;
           ref_medio_y_max=111.4364;
           ref_medio_z_max= 83.1581 ;

           ref_anulare_x_max=14.2326;
           ref_anulare_y_max=86.9458   ;
           ref_anulare_z_max=43.2427;

           ref_mignolo_x_max=-9.2945;
           ref_mignolo_y_max=82.9328;
           ref_mignolo_z_max=45.1346;
          dita_contatto=5;




          q_pca[0]=0.1895;
          q_pca[1]=0.5961;
          q_pca[2]=0.2596;
          q_pca[3]=0.7187;
          q_pca[4]=0.2747;
          q_pca[5]=0.7831;
          q_pca[6]= 0.6901;
          q_pca[7]=0.6920;
          q_pca[8]=0.3363;

          }

          if(presa==4)
             {
           riferimento=Data(68.3438,  107.9042,   75.1842,   25.4309,  117.7980,   84.2114,         0,  111.1274,   80.0199,    4.7401,   84.1654,   45.0081,  -18.9959 ,  80.7153,   44.5419
);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           q_pca=Data( 0.2037,    0.4543,    0.3790,    0.6504,    0.3786,    0.7242,    0.7299,    0.7293,    0.1104);

          }

          if(presa==5)
             {
           riferimento=Data(71.2061,  106.6979,   74.5204,   24.0599,  102.0906,   64.2361,         0,   99.5527,   60.4258,    5.4398,   85.9281,   47.3265,  -17.3885,   84.1846,   48.5711


);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           q_pca=Data( 0.1871,    0.4283,    0.8873,    0.6503,    0.8444,    0.6562,    0.7033,    0.6621,    0.2366);

          }

          if(presa==6)
             {
           riferimento=Data(70.1691,   99.4264,   86.7714,   24.0574,  106.8219,   75.2041,         0,  106.8083,   72.9435,   20.9480,  106.1161,   56.0035,   -5.9468,   99.3113,   60.3730


);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           q_pca=Data(  0.1448,    0.6260,    0.6397,    0.6614,    0.5726,    0.6692,    0.4952,    0.4658,    0.5767);

          }

          if(presa==7)
             {
           /*ref_pollice_x_max=62.3070;
           ref_pollice_y_max=98.5624;
           ref_pollice_z_max=93.8110;

           ref_indice_x_max=29.4300  ;
           ref_indice_y_max=124.9364 ;
           ref_indice_z_max=   90.7968 ;
           ref_medio_x_max=0;
           ref_medio_y_max=122.0080 ;
           ref_medio_z_max= 87.1459;

           ref_anulare_x_max=28.7212;
           ref_anulare_y_max=101.3773   ;
           ref_anulare_z_max=46.3091;

           ref_mignolo_x_max=3.7840;
           ref_mignolo_y_max=92.7443;
           ref_mignolo_z_max=51.7905;*/
          
           ref_pollice_x_max=100.2488;
           ref_pollice_y_max=92.3890;
           ref_pollice_z_max=45.0156;

           ref_indice_x_max=45.2043;
           ref_indice_y_max=178.7556;
           ref_indice_z_max=55.6749;

           ref_medio_x_max=0;
           ref_medio_y_max=187.6427 ;
           ref_medio_z_max= 48.7940;

           ref_anulare_x_max=-39.9169;
           ref_anulare_y_max=176.2222;
           ref_anulare_z_max=50.5568;

           ref_mignolo_x_max=-74.8804;
           ref_mignolo_y_max=147.8797;
           ref_mignolo_z_max=40.8494;

          dita_contatto=5;
            
          // Pseudo Invera
         /* q_pca[0]= 0.1550;
          q_pca[1]=0.7928;
          q_pca[2]= 0.0615;
          q_pca[3]=0.7550;
          q_pca[4]=0.0639;
          q_pca[5]=0.7985;
          q_pca[6]= 0.5359;
          q_pca[7]=0.5487;
          q_pca[8]=0.5767;*/

          // Trasposta
          q_pca[0]= 0.0970;
          q_pca[1]= 0.4461;
          q_pca[2]= 0.4691;
          q_pca[3]= 0.7463;
          q_pca[4]= 0.6295;
          q_pca[5]= 0.7985;
          q_pca[6]= 0.7833;
          q_pca[7]= 0.7408;
          q_pca[8]= 0.0499;

          }

          if(presa==8)
             {


           ref_pollice_x_max=70.3104;
           ref_pollice_y_max=108.4663;
           ref_pollice_z_max=72.0678;

           ref_indice_x_max=24.9784 ;
           ref_indice_y_max=109.5117 ;
           ref_indice_z_max= 75.2226  ;
           ref_medio_x_max=0;
           ref_medio_y_max=105.2838;
           ref_medio_z_max= 71.2092;

           ref_anulare_x_max=1.8130;
           ref_anulare_y_max=84.5230   ;
           ref_anulare_z_max=46.9900;

           ref_mignolo_x_max=-21.5204;
           ref_mignolo_y_max= 81.7312;
           ref_mignolo_z_max=46.1873;
          dita_contatto=5;

           q_pca=Data(0.2003,    0.3902,    0.6412,    0.6199,    0.6137,    0.6683,    0.7237,    0.7049,    0.0885);

          }

          if(presa==9)
             {
           riferimento=Data(63.0777,   90.0939,  101.8071,   32.2058,  134.2953,   88.7042,         0,  135.2089,   85.6346,   27.3675,  146.0656,   54.7361,   -0.8278,  124.5295,   68.8760);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           //Pseudo Inversa
           //q_pca=Data(0.0970,    0.8764,    0.0474,    0.6512,         0,    0.7021,    0.2179,    0.2269,    0.5767);

           // Trasposta
           q_pca=Data(0.1082,    0.5203,    0.1175,    0.6446,         0.0069,    0.6499,    0.3105,    0.3302,    0.5767);
          }

          if(presa==10)
             {
           riferimento=Data(65.0153,  106.3983,   80.8485,   25.8530,  117.4266,   88.1504,         0,  109.8895,   83.0798,    9.3442,   82.4603,   38.7240,  -13.7363,   79.6762,   41.0074




);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           q_pca=Data(  0.2038,    0.5721 ,   0.2411,    0.7394,    0.2779,    0.7985,    0.7637,    0.7676,    0.2287);

          }

          if(presa==11)
             {
           riferimento=Data(68.3657,  108.1804,   74.6369,   25.6867,  132.6930,   87.4541,         0,  124.0667,   85.1685,    5.1178,   86.3262,   48.1795,  -19.5893,   81.2540,   45.7587




);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           q_pca=Data(  0.2053,    0.4447,    0.1518,    0.6122,    0.1520,    0.7215,    0.6958,    0.7150,    0.0605);

          }if(presa==12)
          {
        riferimento=Data(67.0475,  108.4828,   75.3254,   25.4828,  128.8269,   88.1891,         0,  119.2541,   85.0923,    4.0841,   83.2634,   43.7701,  -20.2387,   79.4917,   42.7400




);

        ref_pollice_x_max=riferimento[0];
        ref_pollice_y_max=riferimento[1];
        ref_pollice_z_max=riferimento[2];

        ref_indice_x_max=riferimento[3];
        ref_indice_y_max=riferimento[4];
        ref_indice_z_max= riferimento[5];
        ref_medio_x_max=riferimento[6];
        ref_medio_y_max=riferimento[7];
        ref_medio_z_max= riferimento[8];

        ref_anulare_x_max=riferimento[9];
        ref_anulare_y_max=riferimento[10];
        ref_anulare_z_max=riferimento[11];

        ref_mignolo_x_max=riferimento[12];
        ref_mignolo_y_max= riferimento[13];
        ref_mignolo_z_max=riferimento[14];
       dita_contatto=5;

        q_pca=Data(  0.2115,    0.4643,    0.1631,    0.6520,    0.1820,    0.7547,    0.7455,    0.7622,    0.0513);

       }

          if(presa==13)
             {
           riferimento=Data(63.2243,   97.9560,   93.9002,   30.2005,  127.5340,   90.3910,         0,  124.0034,   87.2593,   29.0545,  106.1848,   49.3646,    3.4557,   95.8734,   54.9002




);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=4;

           q_pca=Data(  0.1492,    0.7795,    0.0566,    0.7294,    0.0468,    0.7917,    0.4946,    0.5053,    0.5767
);

          }

          if(presa==14)
             {
           riferimento=Data(61.6112,  104.4060,   86.5830,   27.0390,  120.6635,   91.1758,         0,  117.2691,   86.0570,   15.1442,   82.8871,   34.4622,   -7.4063,   80.2429,   38.7229);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=3;

           q_pca=Data(  0.1982,    0.6918,    0.0849,    0.7985,    0.1477,    0.7985,    0.7597,    0.7700,    0.3779
);

          }

          if(presa==15)
             {
           riferimento=Data(69.5264,   86.8035,  100.1767,   34.0099,  140.3781,   84.5223,         0,  148.0384,   81.0946,    4.5267,  174.6209,   51.5167,  -25.9522,  145.9520,   69.8941




);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2;

           q_pca=Data(  0.0669,    0.8050,    0.1463,    0.5039,         0,    0.5588,    0.0154,    0.0223,    0.5767
);

          }




          if(presa==16)
             {
           riferimento=Data(62.5934,  108.4339,   79.2389,   24.6066,  103.9921,   82.6458,         0,   98.9903,   74.6921,    1.1296,   78.1557,   27.8299,  -19.9985,   77.5416,   33.7640);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2;

           //Pseudo Inversa
           //q_pca=Data(0.2249,    0.5640,    0.4325,    0.7985,    0.5096,    0.7985,    0.9233,    0.9100,    0.1308);
           
          // Trasposta
          q_pca=Data(0.3316,    0.3596,    0.5582,    0.7425,    0.7701,    0.7985,    0.8432,    0.8295,    0.0);
          }

          if(presa==17)
             {
           riferimento=Data(65.4863,  103.8252,  84.5473,   29.2198,  132.8395,   89.7891,         0,  124.8859,   87.3410,   17.8442,   91.2938,   46.3076,   -6.7021,   84.8909,   47.1529




);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2;

           q_pca=Data(  0.1849,    0.6273,    0.0181,    0.6965,    0.0338,    0.7909,    0.6350,    0.6559,    0.3654
);

          }


          if(presa==18)
             {
           riferimento=Data(64.9763,  107.3778,   79.1947,   25.5644,  116.9586,   87.8753,         0,  108.9393,   82.5663,    6.3497,   81.0358,   37.4027,  -16.6800,   78.6973,   39.6582);


           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2;
           // Pinv
           q_pca=Data(0.2106,    0.5448,    0.2535,    0.7377,    0.2957,    0.7985,    0.7947,    0.7988,    0.1619);
                
          // Trasposta
            //q_pca=Data(0.2419,    0.3750,    0.4972,    0.6909,    0.6731,    0.7707,    0.7621,    0.7414,    0.0189);
          }

          if(presa==19)
             {
           riferimento=Data(61.6466,  106.8721 ,  82.6750,   23.7244,  100.5195,   78.8545,         0,   95.4255,   70.1314,    4.5140,   78.3595,   25.7978,  -15.4533,   77.9828,   33.3151);


           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2;

           q_pca=Data(0.2162,    0.6300,    0.5300,    0.7985,    0.6098,    0.7985 ,   0.9183,    0.8933,    0.2675);

          }

          if(presa==20)
             {
           riferimento=Data(68.6706,  101.3040,   85.5457,   31.0353,  133.7091,   87.9325,         0,  130.3227,   85.7241,   20.6308,  106.2208,   56.2607,   -5.4264,   95.5547,   57.2413);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           q_pca=Data(0.1604,    0.6185,    0.1044,    0.6230,    0.0673,    0.7098,    0.4946,    0.5103,    0.4985);

          }

          if(presa==21)
             {
           riferimento=Data(60.9725,  100.0695,   92.8754,   28.9654,  123.3699,   91.5488,         0,  122.1140,   87.1636,   26.5215,   93.6506,   40.8361,    2.7039 ,  87.5894 ,  45.9449
);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=4;

           //q_pca=Data(0.1681,    0.7873,    0.0361,    0.7940,    0.0673,    0.7985,    0.6115,    0.6222,    0.5767);
           q_pca=Data(0.3977,    0.3326,    0.3700,    0.6816,    0.4376,    0.7382,    0.5874,    0.6057,    0.5485);
          }

          if(presa==22)
             {
           riferimento=Data(59.3628,  105.2706,   86.8088,   27.5570,  125.3768,   91.7990,         0,  122.5040 ,  87.2261,   12.2797,   80.2830,   29.4287,   -9.7615,   78.6257,   34.8739
);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=3;

           q_pca=Data( 0.2098,    0.7172,         0,    0.7985 ,   0.0609,    0.7985,    0.8227,    0.8408,    0.3296);
           
          }

          if(presa==23)
             {
           riferimento=Data(61.0236,   91.7298,  101.6160,   30.8066,  129.5775,   90.0555,         0,  130.2491,   86.8567,   32.9554,  131.9992,   52.3572,    5.7444,  114.4700,   64.5253
);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2;

           //q_pca=Data( 0.1105,    0.8933,    0.0474,    0.7102,         0,    0.7558,    0.3097,    0.3168,    0.5767);
           q_pca=Data( 0.4703,    0.2042,    0.2112,    0.6547,         0.1657,    0.6791,    0.4089,    0.4304,    0.9338);
          }


          if(presa==24)
             {
           riferimento=Data(69.2141,   87.9312,   99.4892,   34.6726,  142.6123,   84.7210,         0,  146.4473,   81.7724,    7.9601,  170.4752,   53.5671,  -22.0638,  142.4161,   70.8197);

           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           //q_pca=Data(0.0743,    0.7991,    0.0909,    0.5136,         0,    0.5770,    0.0485,    0.0600,    0.5767);

          // Trasposta
            q_pca=Data(0.0708,    0.4558,    0.0373,    0.4480,         0,    0.4334,    0.1256,    0.1493,    0.5767);

          }

          if(presa==25)
             {
           riferimento=Data(60.7301,   92.5670,  101.0272,   30.7900,  129.5218,   90.3615,         0,  128.8043,   87.1600,   33.7476,  127.1941,   51.5202,    6.9144,  110.8497,   62.7320);


           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=4;

           q_pca=Data(0.1165,    0.8895,    0.0292,    0.7231,         0,    0.7714,    0.3416,    0.3501,    0.5767);


          }
          if(presa==26)
             {
           riferimento=Data(62.9861,   90.1008,  101.8579,   31.8919,  133.2370,   88.7985,         0,  135.0641,   85.6743,   27.7130,  145.4318,   54.6513,   -0.5222,  124.2070,   68.7527);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;
            
           // Pseudo Inversa
           q_pca=Data(0.1027,    0.8829,    0.0615,    0.6538,         0,    0.7015,    0.2227,    0.2320,    0.5767);

          // Trasposta
           //q_pca=Data(0.1275,    0.5091,    0.1139,    0.6287,         0.0003,    0.6332,    0.2947,    0.3185,    0.5767); 
}
          if(presa==27)
             {
           riferimento=Data(80.9198,  96.3250,   79.7491,   26.1002,  115.0240,   85.4879,         0,  107.0414,   78.9708,   10.9038,   93.3195,   52.9888,   -13.3927,   89.0652,   52.8107);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=3;

           // Trasposta 
           q_pca=Data(0.0997,  0.4701,   0.3443,    0.7051,    0.4090,    0.7545,    0.6132,    0.5925,    0.4312);
}

          if(presa==28)
             {
           riferimento=Data(62.5502,   94.0116,   98.5381,   30.7977,  129.5477,   89.9574,         0,  129.1908,   86.9072,   31.9153,  123.4799,   52.9248,    4.9284,  108.1606,   62.3194);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];           
          dita_contatto=2;

           //q_pca=Data(0.8434,    0.1235,    0.0537,    0.7064,    0.0115,    0.7595,    0.3668,    0.3755,    0.9527);
        
        // Trasposta
        q_pca=Data( 0.0754,   0.4909,    0.2357,    0.6577 ,   0.2205,    0.6852,    0.4646,    0.4576,    0.5766);
}

          if(presa==29)
             {
           riferimento=Data(65.2524,   86.9754,  102.9765 ,  32.7988 , 136.2946 ,  86.8675,         0,  141.8978,   83.5261,   15.7219,  165.5525,   53.0101,  -13.6137 , 139.3387,   71.1323);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2;

           q_pca=Data(0.0742,    0.8706,    0.1104,    0.5825,         0,    0.6283,    0.0853,    0.0908,    0.5767);

}

          if(presa==30)
             {
           riferimento=Data(64.6376,   93.1665,   98.0036 ,  32.4457,  135.1042,   88.6918,         0,  134.1082 ,  85.9304,   28.3461 , 134.0443,   56.3858,    0.4297,  115.3353,   66.5416);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5;

           q_pca=Data(0.1147,    0.8182,    0.0322,    0.6514,         0,    0.7141,    0.2963,    0.3089,    0.5767);

}
          if(presa==31)
             {
           riferimento=Data(61.8632,  105.6675,   84.4717,   25.7810 , 115.0863 ,  89.5287,         0,  110.8146,   83.5488,   10.6273,   80.4983,   31.9823,  -11.3007,   78.8391,   36.8618
);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=2; //??????

           q_pca=Data(0.2068,    0.6569,    0.1914,    0.7985,    0.2608,    0.7985,    0.8146 ,   0.8174,    0.3047);

}
          if(presa==32)
             {
           riferimento=Data(85.3597,   99.6800,   68.3018,   36.0637,  157.6808,   71.0565 ,        0,  167.9797,   68.8366 , -19.9488 , 176.5988,   54.1067 , -52.1793,  148.6475,   56.2178

);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5; //??????

           q_pca=Data(0.1021,    0.2924,    0.2822 ,   0.1608 ,   0.0245,    0.2958   , 0.0086,    0.0328,    0.4560);

}
          if(presa==33)
             {
           riferimento=Data(85.3328,  106.7096,   51.6582,   26.1449,  168.9506,   62.3114,         0,  177.4077 ,  60.9970,  -17.7070,  176.3918,   53.5239,  -40.2634,  151.3336,   48.0409

);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5; //??????

           q_pca=Data(0.1248,    0.0396,    0.2779,         0,         0 ,   0.1821,    0.0256,    0.0641,    0.0388);

}

          if(presa==34)
             {
           riferimento=Data(84.5358,  107.3648,   51.6107,   25.0000,  171.9505,   61.4232,         0,  176.3292,   62.0649,  -16.3208,  173.1831,   56.3774 , -38.0531 , 147.8532,   50.4756


);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5; //??????

           q_pca=Data(0.1307,    0.0369,    0.2153,    0.0046,         0,    0.1981 ,   0.0499,    0.0935,         0
);

}
          if(presa==35)
             {
           riferimento=Data(81.5163,   93.8977,   82.1308 ,  35.9729,  146.9965,   77.2907 ,        0,  157.7436,   74.9111,  -12.8168,  175.8090,   55.0601,  -47.0124,  146.4300,   64.1080



);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5; //??????

           q_pca=Data(0.0848,    0.5019,    0.2950 ,   0.3022 ,   0.0673 ,   0.3947,    0.0055,    0.0170,    0.5767
);

}
          if(presa==36)
             {
           riferimento=Data(72.4999,   87.1501,   97.6200,   33.5014,  138.6634,   82.9698,         0,  149.0536,   79.8396 ,   0.0866 , 176.4524,   51.5417,  -32.1267,  147.9153,   68.5159



);



           ref_pollice_x_max=riferimento[0];
           ref_pollice_y_max=riferimento[1];
           ref_pollice_z_max=riferimento[2];

           ref_indice_x_max=riferimento[3];
           ref_indice_y_max=riferimento[4];
           ref_indice_z_max= riferimento[5];
           ref_medio_x_max=riferimento[6];
           ref_medio_y_max=riferimento[7];
           ref_medio_z_max= riferimento[8];

           ref_anulare_x_max=riferimento[9];
           ref_anulare_y_max=riferimento[10];
           ref_anulare_z_max=riferimento[11];

           ref_mignolo_x_max=riferimento[12];
           ref_mignolo_y_max= riferimento[13];
           ref_mignolo_z_max=riferimento[14];
          dita_contatto=5; //??????

           q_pca=Data(0.0643,    0.7536,    0.2378 ,   0.4616,    0.0467,    0.5138,         0,         0,    0.5767

);

}



  // Main loop.

              if (correzione==true && standard==false && syn==false)
                  {
                  if(getchar()){


  while (nh.ok() )
  {

          for(size_t channel = 0; channel < 9; ++channel)
          {
             channel_pos.position[channel]=svh_node.getCurrentPositions().position[channel];
                                std::ofstream ck1;
                                ck1.open("/home/fanny/catkin_ws/cp.txt",std::ios_base::app);
                                if(!ck1){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                ck1 << svh_node.getCurrentPositions().position[channel] << " "; 
                                ck1.close();
        
             if(channel==0)
             {
                           m[0][1]=(svh_node.getCurrentPositions().position[channel]*180)/PI;
             }else
             {
                  if(channel==1)
                  {
                                m[0][0]=(svh_node.getCurrentPositions().position[channel]*180)/PI;                   
                  }
                  else
                  {
                                m[0][channel]=(svh_node.getCurrentPositions().position[channel]*180)/PI;}
                  }
             } 
                        
                                std::ofstream ck1;
                                ck1.open("/home/fanny/catkin_ws/cp.txt",std::ios_base::app);
                                if(!ck1){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                ck1 << " " << endl; 
                                ck1.close();


      //Pollice
      q1=0.5*(m[0][0]);
      q2=0.29*(m[0][1])+20.7941;
      q3=0.29*(m[0][1])+180-161.8544;
      q4=0.42*(m[0][1])+180-176.4734;

      //indice

      q10=0.49*(m[0][2])+180-161.8584;
      q6a=m[0][3]+23.7532;
      q6b=0.25*(m[0][8]-66.08);
      q14=0.51*(m[0][2])+180-168.8171;

      //medio
      q11=0.49*(m[0][4])+180-161.7631;
      q7=m[0][5]+23.7532;
      q15=0.51*(m[0][4])+180-168.1028;
 
      //opposizione 
      q5=0.5*(m[0][0]);

      //anulare
      q8b=0.25*(m[0][8]-66.08);
      q8a=0.26*(m[0][6])+25.641;
      q12=0.36*(m[0][6])+180-161.7631;
      q16=0.38*(m[0][6])+180-168.1028;

      //mignolo
      q9b=0.5*(m[0][8]-66.08);

      q9a=0.26*(m[0][7])+25.641;
      q13=0.36*(m[0][7])+180-162.0458;
      q17=0.38*(m[0][7])+180-169.6355;

          sq1=sin(Rad*q1);
          sq2=sin(Rad*q2);
          sq3=sin(Rad*q3);
          sq4=sin(Rad*q4);
          sq5=sin(Rad*q5);
          sq6a=sin(Rad*q6a);
          sq6b=sin(Rad*q6b);
          sq7=sin(Rad*q7);
          sq8a=sin(Rad*q8a);
          sq8b=sin(Rad*(q8b-90));
          sq9a=sin(Rad*q9a);
          sq9b=sin(Rad*(q9b-90));
          sq10=sin(Rad*q10);
          sq11=sin(Rad*q11);
          sq12=sin(Rad*q12);
          sq13=sin(Rad*q13);
          sq14=sin(Rad*q14);
          sq15=sin(Rad*q15);
          sq16=sin(Rad*q16);
          sq17=sin(Rad*q17);


          cq1=cos(Rad*q1);
          cq2=cos(Rad*q2);
          cq3=cos(Rad*q3);
          cq4=cos(Rad*q4);
          cq5=cos(Rad*q5);
          cq6a=cos(Rad*q6a);
          cq6b=cos(Rad*q6b);
          cq7=cos(Rad*q7);
          cq8a=cos(Rad*q8a);
          cq8b=cos(Rad*(q8b-90));
          cq9a=cos(Rad*q9a);
          cq9b=cos(Rad*(q9b-90));
          cq10=cos(Rad*q10);
          cq11=cos(Rad*q11);
          cq12=cos(Rad*q12);
          cq13=cos(Rad*q13);
          cq14=cos(Rad*q14);
          cq15=cos(Rad*q15);
          cq16=cos(Rad*q16);
          cq17=cos(Rad*q17);

          DH_TP1=Data(cq1, -sq1*cos(Rad*DH_P(0,1)), sq1*sin(Rad*DH_P(0,1)), DH_P(0,0)*cq1,
                      sq1,  cq1*cos(Rad*DH_P(0,1)), -cq1*sin(Rad*DH_P(0,1)), DH_P(0,0)*sq1,
                      0,    sin(Rad*DH_P(0,1)),     cos(Rad*DH_P(0,1)),      DH_P(0,2),
                      0,     0,                           0,                  1

                      );

          DH_TP2=Data(cq2, -sq2*cos(Rad*DH_P(1,1)), sq2*sin(Rad*DH_P(1,1)), DH_P(1,0)*cq2,
                      sq2,  cq2*cos(Rad*DH_P(1,1)), -cq2*sin(Rad*DH_P(1,1)), DH_P(1,0)*sq2,
                      0,    sin(Rad*DH_P(1,1)),     cos(Rad*DH_P(1,1)),      DH_P(1,2),
                      0,     0,                           0,                  1

                      );

          DH_TP3=Data(cq3, -sq3*cos(Rad*DH_P(2,1)), sq3*sin(Rad*DH_P(2,1)), DH_P(2,0)*cq3,
                      sq3,  cq3*cos(Rad*DH_P(2,1)), -cq3*sin(Rad*DH_P(2,1)), DH_P(2,0)*sq3,
                      0,    sin(Rad*DH_P(2,1)),     cos(Rad*DH_P(2,1)),      DH_P(2,2),
                      0,     0,                           0,                  1

                      );
          DH_TP4=Data(cq4, -sq4*cos(Rad*DH_P(3,1)), sq4*sin(Rad*DH_P(3,1)), DH_P(3,0)*cq4,
                      sq4,  cq4*cos(Rad*DH_P(3,1)), -cq4*sin(Rad*DH_P(3,1)), DH_P(3,0)*sq4,
                      0,    sin(Rad*DH_P(3,1)),     cos(Rad*DH_P(3,1)),      DH_P(3,2),
                      0,     0,                           0,                  1
                      );


          DH_TI1=Data(cq6b, -sq6b*cos(Rad*DH_I(0,1)), sq6b*sin(Rad*DH_I(0,1)), DH_I(0,0)*cq6b,
                      sq6b,  cq6b*cos(Rad*DH_I(0,1)), -cq6b*sin(Rad*DH_I(0,1)), DH_I(0,0)*sq6b,
                      0,      sin(Rad*DH_I(0,1)),       cos(Rad*DH_I(0,1)),          DH_I(0,2),
                      0,     0,                        0,                        1

                      );

          DH_TI2=Data(cq6a, -sq6a*cos(Rad*DH_I(1,1)), sq6a*sin(Rad*DH_I(1,1)), DH_I(1,0)*cq6a,
                      sq6a,  cq6a*cos(Rad*DH_I(1,1)), -cq6a*sin(Rad*DH_I(1,1)), DH_I(1,0)*sq6a,
                      0,      sin(Rad*DH_I(1,1)),       cos(Rad*DH_I(1,1)),          DH_I(1,2),
                      0,     0,                        0,                        1

                      );

          DH_TI3=Data(cq10, -sq10*cos(Rad*DH_I(2,1)), sq10*sin(Rad*DH_I(2,1)), DH_I(2,0)*cq10,
                      sq10,  cq10*cos(Rad*DH_I(2,1)), -cq10*sin(Rad*DH_I(2,1)), DH_I(2,0)*sq10,
                      0,      sin(Rad*DH_I(2,1)),       cos(Rad*DH_I(2,1)),          DH_I(2,2),
                      0,     0,                        0,                        1

                      );

          DH_TI4=Data(cq14, -sq14*cos(Rad*DH_I(3,1)), sq14*sin(Rad*DH_I(3,1)), DH_I(3,0)*cq14,
                      sq14,  cq14*cos(Rad*DH_I(3,1)), -cq14*sin(Rad*DH_I(3,1)), DH_I(3,0)*sq14,
                      0,      sin(Rad*DH_I(3,1)),       cos(Rad*DH_I(3,1)),          DH_I(3,2),
                      0,     0,                        0,                        1

                      );

          DH_MI1=Data(cq7, -sq7*cos(Rad*DH_M(0,1)), sq7*sin(Rad*DH_M(0,1)), DH_M(0,0)*cq7,
                      sq7,  cq7*cos(Rad*DH_M(0,1)), -cq7*sin(Rad*DH_M(0,1)), DH_M(0,0)*sq7,
                      0,    sin(Rad*DH_M(0,1)),      cos(Rad*DH_M(0,1)),         DH_M(0,2),
                      0,    0,                       0,                      1

                      );

          DH_MI2=Data(cq11, -sq11*cos(Rad*DH_M(1,1)), sq11*sin(Rad*DH_M(1,1)), DH_M(1,0)*cq11,
                      sq11,  cq11*cos(Rad*DH_M(1,1)), -cq11*sin(Rad*DH_M(1,1)), DH_M(1,0)*sq11,
                      0,    sin(Rad*DH_M(1,1)),      cos(Rad*DH_M(1,1)),         DH_M(1,2),
                      0,    0,                       0,                      1

                      );

          DH_MI3=Data(cq15, -sq15*cos(Rad*DH_M(2,1)), sq15*sin(Rad*DH_M(2,1)), DH_M(2,0)*cq15,
                      sq15,  cq15*cos(Rad*DH_M(2,1)), -cq15*sin(Rad*DH_M(2,1)), DH_M(2,0)*sq15,
                       0,    sin(Rad*DH_M(2,1)),      cos(Rad*DH_M(2,1)),         DH_M(2,2),
                       0,    0,                       0,                      1

                      );
          DH_TA1=Data(cq5, -sq5*cos(Rad*DH_A(0,1)), sq5*sin(Rad*DH_A(0,1)), DH_A(0,0)*cq5,
                      sq5,  cq5*cos(Rad*DH_A(0,1)), -cq5*sin(Rad*DH_A(0,1)),    DH_A(0,0)*sq5,
                      0,    sin(Rad*DH_A(0,1)),     cos(Rad*DH_A(0,1)),     DH_A(0,2),
                      0,    0,                           0,                     1

                      );


          DH_TA2=Data(cq8b, -sq8b*cos(Rad*DH_A(1,1)), sq8b*sin(Rad*DH_A(1,1)), DH_A(1,0)*cq8b,
                      sq8b,  cq8b*cos(Rad*DH_A(1,1)), -cq8b*sin(Rad*DH_A(1,1)),    DH_A(1,0)*sq8b,
                      0,    sin(Rad*DH_A(1,1)),     cos(Rad*DH_A(1,1)),     DH_A(1,2),
                      0,    0,                           0,                     1

                      );

          DH_TA3=Data(cq8a, -sq8a*cos(Rad*DH_A(2,1)), sq8a*sin(Rad*DH_A(2,1)), DH_A(2,0)*cq8a,
                      sq8a,  cq8a*cos(Rad*DH_A(2,1)), -cq8a*sin(Rad*DH_A(2,1)),    DH_A(2,0)*sq8a,
                      0,    sin(Rad*DH_A(2,1)),     cos(Rad*DH_A(2,1)),     DH_A(2,2),
                      0,    0,                           0,                     1

                      );

          DH_TA4=Data(cq12, -sq12*cos(Rad*DH_A(3,1)), sq12*sin(Rad*DH_A(3,1)), DH_A(3,0)*cq12,
                      sq12,  cq12*cos(Rad*DH_A(3,1)), -cq12*sin(Rad*DH_A(3,1)),    DH_A(3,0)*sq12,
                      0,    sin(Rad*DH_A(3,1)),     cos(Rad*DH_A(3,1)),     DH_A(3,2),
                      0,    0,                           0,                     1

                      );

          DH_TA5=Data(cq16, -sq16*cos(Rad*DH_A(4,1)), sq16*sin(Rad*DH_A(4,1)), DH_A(4,0)*cq16,
                      sq16,  cq16*cos(Rad*DH_A(4,1)), -cq16*sin(Rad*DH_A(4,1)),    DH_A(4,0)*sq16,
                      0,    sin(Rad*DH_A(4,1)),     cos(Rad*DH_A(4,1)),     DH_A(4,2),
                      0,    0,                           0,                     1

                      );

          DH_TMI1=Data(cq5, -sq5*cos(Rad*DH_MI(0,1)), sq5*sin(Rad*DH_MI(0,1)), DH_MI(0,0)*cq5,
                      sq5, cq5*cos(Rad*DH_MI(0,1)), -cq5*sin(Rad*DH_MI(0,1)), DH_MI(0,0)*sq5,
                      0,   sin(Rad*DH_MI(0,1)),      cos(Rad*DH_MI(0,1)),     DH_MI(0,2),
                      0,   0,                             0,                   1

                      );


          DH_TMI2=Data(cq9b, -sq9b*cos(Rad*DH_MI(1,1)), sq9b*sin(Rad*DH_MI(1,1)), DH_MI(1,0)*cq9b,
                      sq9b, cq9b*cos(Rad*DH_MI(1,1)), -cq9b*sin(Rad*DH_MI(1,1)), DH_MI(1,0)*sq9b,
                      0,   sin(Rad*DH_MI(1,1)),      cos(Rad*DH_MI(1,1)),     DH_MI(1,2),
                      0,   0,                             0,                   1

                      );


          DH_TMI3=Data(cq9a, -sq9a*cos(Rad*DH_MI(2,1)), sq9a*sin(Rad*DH_MI(2,1)), DH_MI(2,0)*cq9a,
                      sq9a, cq9a*cos(Rad*DH_MI(2,1)), -cq9a*sin(Rad*DH_MI(2,1)), DH_MI(2,0)*sq9a,
                      0,   sin(Rad*DH_MI(2,1)),      cos(Rad*DH_MI(2,1)),     DH_MI(2,2),
                      0,   0,                             0,                   1

                      );

          DH_TMI4=Data(cq13, -sq13*cos(Rad*DH_MI(3,1)), sq13*sin(Rad*DH_MI(3,1)), DH_MI(3,0)*cq13,
                      sq13, cq13*cos(Rad*DH_MI(3,1)), -cq13*sin(Rad*DH_MI(3,1)), DH_MI(3,0)*sq13,
                      0,   sin(Rad*DH_MI(3,1)),      cos(Rad*DH_MI(3,1)),     DH_MI(3,2),
                      0,   0,                             0,                   1

                      );

          DH_TMI5=Data(cq17, -sq17*cos(Rad*DH_MI(4,1)), sq17*sin(Rad*DH_MI(4,1)), DH_MI(4,0)*cq17,
                      sq17, cq17*cos(Rad*DH_MI(4,1)), -cq17*sin(Rad*DH_MI(4,1)), DH_MI(4,0)*sq17,
                      0,   sin(Rad*DH_MI(4,1)),      cos(Rad*DH_MI(4,1)),     DH_MI(4,2),
                      0,   0,                             0,                   1

                      );

          //Cinematica diretta del POLLICE
          H_p0 = TP0*DH_TP1;
          H_p1 = H_p0*DH_TP2;
          H_p2 = H_p1*DH_TP3;
          H_p3 = H_p2*DH_TP4;
          H_pe = H_p3;


          //Cinematica diretta dell' INDICE
          H_i0 = TI0*DH_TI1;
          H_i1 = H_i0*DH_TI2;
          H_i2 = H_i1*DH_TI3;
          H_i3 = H_i2*DH_TI4;
          H_ie = H_i3;


          //Cinematica diretta del MEDIO
          H_m0 = TM0*DH_MI1;
          H_m1 = H_m0*DH_MI2;
          H_m2 = H_m1*DH_MI3;
          H_me = H_m2;


          //Cinematica diretta dell' ANULARE
          H_a0 = TA0*DH_TA1;
          H_a1 = H_a0*DH_TA2;
          H_a2 = H_a1*DH_TA3;
          H_a3 = H_a2*DH_TA4;
          H_a4 = H_a3*DH_TA5;
          H_ae = H_a4;


          //Cinematica diretta del MIGNOLO
          H_mi0 = TMI0*DH_TMI1;
          H_mi1 = H_mi0*DH_TMI2;
          H_mi2 = H_mi1*DH_TMI3;
          H_mi3 = H_mi2*DH_TMI4;
          H_mi4 = H_mi3*DH_TMI5;
          H_mie = H_mi4;

          for(int i=0; i<3; i++)
          {
                  a[i]=TP0[i][2];
          }

          for(int i=0; i<3; i++)
          {
                  b_temp1[i]=H_pe[i][3];
          }
                  
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=TP0[i][3];
          }
              
          b=b_temp1-b_temp2;

          Vector<3> cross0 =a ^ b;

          for(int i=0;i<3;i++)
          {
                  Jp[i][0]=cross0[i];
          }
              
          for(int i=0;i<3;i++)
          {
                  a[i]=H_p0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_p0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_p1[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_p1[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross1 =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_p2[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_p2[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross2 =a ^ b;

          for(int i=0;i<3;i++)
          {
                  Jp[i][1]=cross[i]+cross1[i]+cross2[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=TI0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp1[i]=H_ie[i][3];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=TI0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross0_ind =a ^ b;
          
          for(int i=0;i<3;i++)
          {
                  Ji[i][0]=cross0_ind[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=H_i0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {       
                  b_temp2[i]=H_i0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross_ind =a ^ b;
          
          for(int i=0;i<3;i++)
          {
                  Ji[i][1]=cross_ind[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=H_i1[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_i1[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross1_ind =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_i2[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_i2[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross2_ind =a ^ b;

          for(int i=0;i<3;i++)
          {
                  Ji[i][2]=cross1_ind[i]+cross2_ind[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=TM0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp1[i]=H_me[i][3];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=TM0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross0_med =a ^ b;
          
          for(int i=0;i<3;i++)
          {
                  Jm[i][0]=cross0_med[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=H_m0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_m0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross_med =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_m1[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_m1[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross1_med =a ^ b;

          for(int i=0;i<3;i++)
          {
                  Jm[i][1]=cross_med[i]+cross1_med[i];
          }
          
          //ANULARE

          for(int i=0;i<3;i++)
          {
                  a[i]=TA0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp1[i]=H_ae[i][3];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=TA0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross0_anulare =a ^ b;
          
          for(int i=0;i<3;i++)
          {
                  Ja[i][0]=cross0_anulare[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=H_a0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_a0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross1_anulare =a ^ b;
          
          for(int i=0;i<3;i++)
          {
                  Ja[i][1]=cross1_anulare[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=H_a1[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_a1[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross2_an =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_a2[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_a2[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross3_an =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_a3[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_a3[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross4_an =a ^ b;

          for(int i=0;i<3;i++)
          {
                  Ja[i][2]=cross2_an[i]+cross3_an[i]+cross4_an[i];
          }
          
          //mignolo

          for(int i=0;i<3;i++)
          {
                  a[i]=TMI0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp1[i]=H_mie[i][3];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=TMI0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross0_mi =a ^ b;
          
          for(int i=0;i<3;i++)
          {
                  Jmi[i][0]=cross0_mi[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=H_mi0[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_mi0[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross1_mi =a ^ b;
          
          for(int i=0;i<3;i++)
          {
                  Jmi[i][1]=cross1_mi[i];
          }
          
          for(int i=0;i<3;i++)
          {
                  a[i]=H_mi1[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_mi1[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross2_mi =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_mi2[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_mi2[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross3_mi =a ^ b;

          for(int i=0;i<3;i++)
          {
                  a[i]=H_mi3[i][2];
          }
          
          for(int i=0; i<3; i++)
          {
                  b_temp2[i]=H_mi3[i][3];
          }
          
          b=b_temp1-b_temp2;

          Vector<3> cross4_mi =a ^ b;

          for(int i=0;i<3;i++)
          {
                  Jmi[i][2]=cross2_mi[i]+cross3_mi[i]+cross4_mi[i];
          }

          Fill(J)=Jp[0][0],  Jp[0][1],          0.0,          0.0,          0.0,              0.0,          0.0,              0.0,              0.0,
                    Jp[1][0],  Jp[1][1],          0.0,          0.0,          0.0,              0.0,          0.0,              0.0,              0.0,
                    Jp[2][0],  Jp[2][1],          0.0,          0.0,          0.0,              0.0,          0.0,              0.0,              0.0,
                     0.0,           0.0,      Ji[0][2],         Ji[0][1],          0.0,              0.0,          0.0,              0.0,              Ji[0][0],
                     0.0,           0.0,      Ji[1][2],         Ji[1][1],          0.0,              0.0,          0.0,              0.0,              Ji[1][0],
                     0.0,           0.0,      Ji[2][2],         Ji[2][1],          0.0,              0.0,          0.0,              0.0,              Ji[2][0],
                     0.0,           0.0,          0.0,          0.0,          Jm[0][1],         Jm[0][0],     0.0,              0.0,              0.0,
                     0.0,           0.0,          0.0,          0.0,          Jm[1][1],         Jm[1][0],     0.0,              0.0,              0.0,
                     0.0,           0.0,          0.0,          0.0,          Jm[2][1],         Jm[2][0],     0.0,              0.0,              0.0,
                  Ja[0][0],         0.0,            0.0,          0.0,          0.0,              0.0,       Ja[0][2],              0.0,             Ja[0][1],
                  Ja[1][0],         0.0,            0.0,          0.0,          0.0,              0.0,       Ja[1][2],              0.0,             Ja[1][1],
                  Ja[2][0],         0.0,            0.0,          0.0,          0.0,              0.0,       Ja[2][2],              0.0,             Ja[2][1],
                 Jmi[0][0],         0.0,            0.0,           0.0,          0.0,              0.0,          0.0,              Jmi[0][2],        Jmi[0][1],
                 Jmi[1][0],         0.0,            0.0,           0.0,          0.0,              0.0,          0.0,              Jmi[1][2],        Jmi[1][1],
                 Jmi[2][0],         0.0,            0.0,           0.0,          0.0,              0.0,          0.0,              Jmi[2][2],        Jmi[2][1];
 

         //Pollice        
         for(int i=0;i<3;i++)
         {
                 xp[i] = H_pe[i][3];
         }

         x_mis_pollice=xp[0];
         y_mis_pollice=xp[1];
         z_mis_pollice=xp[2];

         //Indice
         for(int i=0;i<3;i++)
         {
                 xi[i] = H_ie[i][3];
         }

         x_mis_indice=xi[0];
         y_mis_indice=xi[1];
         z_mis_indice=xi[2];

         //Medio
         for(int i=0;i<3;i++)
         {
                 xme[i] = H_me[i][3];
         }

         x_mis_medio=xme[0];
         y_mis_medio=xme[1];
         z_mis_medio=xme[2];


         //Anulare
         for(int i=0;i<3;i++)
         {
                 xa[i] = H_ae[i][3];
         }

         x_mis_anulare=xa[0];
         y_mis_anulare=xa[1];
         z_mis_anulare=xa[2];


         //Mignolo
         for(int i=0;i<3;i++)
         {
                 xmi[i] = H_mie[i][3];
         }

         x_mis_mignolo=xmi[0];
         y_mis_mignolo=xmi[1];
         z_mis_mignolo=xmi[2];


                        std::ofstream ck2;
                        ck2.open("/home/fanny/catkin_ws/posizione.txt",std::ios_base::app);
                        if(!f){
                                cout<<"errore nella creazione del file!";
                                return -1;
                                }

                        ck2 << x_mis_pollice << " " << y_mis_pollice << " " << z_mis_pollice << " " << endl;
                        ck2 << x_mis_indice << " " << y_mis_indice << " " << z_mis_indice << " " << endl;
                        ck2 << x_mis_medio << " " << y_mis_medio << " " << z_mis_medio << " " << endl;                        
                        ck2 << x_mis_anulare << " " << y_mis_anulare << " " << z_mis_anulare << " " << endl;
                        ck2 << x_mis_mignolo << " " << y_mis_mignolo << " " << z_mis_mignolo << " " << endl;        
                        ck2 << " " << endl;
                        ck2.close();




                //Calcolo del vettore delle sinergie correnti
                sigma = E.T()*(m.T() - mean_value);

                        std::ofstream ck3;
                        ck3.open("/home/fanny/catkin_ws/sigma.txt",std::ios_base::app);
                        if(!f){
                                cout<<"errore nella creazione del file!";
                                return -1;
                                }

                        ck3 << sigma << endl;
                        ck3 << " " << endl;
                        ck3.close();
                             

                

        
     // Aggiorno il riferimento
   if(count < 100*delta_t)
   {
     ref_pollice_x = ((ref_pollice_x_max - ref_pollice_x_0)/delta_t)*(count*dt) + ref_pollice_x_0; 
     ref_pollice_y = ((ref_pollice_y_max - ref_pollice_y_0)/delta_t)*(count*dt) + ref_pollice_y_0; 
     ref_pollice_z = ((ref_pollice_z_max - ref_pollice_z_0)/delta_t)*(count*dt) + ref_pollice_z_0;

     ref_indice_x = ((ref_indice_x_max - ref_indice_x_0)/delta_t)*(count*dt) + ref_indice_x_0;
     ref_indice_y = ((ref_indice_y_max - ref_indice_y_0)/delta_t)*(count*dt) + ref_indice_y_0; 
     ref_indice_z = ((ref_indice_z_max - ref_indice_z_0)/delta_t)*(count*dt) + ref_indice_z_0;

     ref_medio_x = ((ref_medio_x_max - ref_medio_x_0)/delta_t)*(count*dt) + ref_medio_x_0;
     ref_medio_y = ((ref_medio_y_max - ref_medio_y_0)/delta_t)*(count*dt) + ref_medio_y_0; 
     ref_medio_z = ((ref_medio_z_max - ref_medio_z_0)/delta_t)*(count*dt) + ref_medio_z_0;

     ref_anulare_x = ((ref_anulare_x_max - ref_anulare_x_0)/delta_t)*(count*dt) + ref_anulare_x_0; 
     ref_anulare_y = ((ref_anulare_y_max - ref_anulare_y_0)/delta_t)*(count*dt) + ref_anulare_y_0;
     ref_anulare_z = ((ref_anulare_z_max - ref_anulare_z_0)/delta_t)*(count*dt) + ref_anulare_z_0;

     ref_mignolo_x = ((ref_mignolo_x_max - ref_mignolo_x_0)/delta_t)*(count*dt) + ref_mignolo_x_0;
     ref_mignolo_y = ((ref_mignolo_y_max - ref_mignolo_y_0)/delta_t)*(count*dt) + ref_mignolo_y_0;
     ref_mignolo_z = ((ref_mignolo_z_max - ref_mignolo_z_0)/delta_t)*(count*dt) + ref_mignolo_z_0;
   }else
   {
     ref_pollice_x = ref_pollice_x_max;
     ref_pollice_y = ref_pollice_y_max; 
     ref_pollice_z = ref_pollice_z_max;

     ref_indice_x = ref_indice_x_max; 
     ref_indice_y = ref_indice_y_max; 
     ref_indice_z = ref_indice_z_max;

     ref_medio_x = ref_medio_x_max; 
     ref_medio_y = ref_medio_y_max; 
     ref_medio_z = ref_medio_z_max;

     ref_anulare_x = ref_anulare_x_max; 
     ref_anulare_y = ref_anulare_y_max;
     ref_anulare_z = ref_anulare_z_max;

     ref_mignolo_x = ref_mignolo_x_max;
     ref_mignolo_y = ref_mignolo_y_max;
     ref_mignolo_z = ref_mignolo_z_max;
   }
    
     count = count + 1;

                        std::ofstream ck8;
                        ck8.open("/home/fanny/catkin_ws/riferimento.txt",std::ios_base::app);
                        if(!ck8){
                                cout<<"errore nella creazione del file!";
                                return -1;
                                 }

                        ck8 << ref_pollice_x << ref_pollice_y << ref_pollice_z << endl;
                        ck8 << ref_indice_x << ref_indice_y << ref_indice_z << endl;
                        ck8 << ref_medio_x << ref_medio_y << ref_medio_z << endl;
                        ck8 << ref_anulare_x << ref_anulare_y << ref_anulare_z << endl;
                        ck8 << ref_mignolo_x << ref_mignolo_y << ref_mignolo_z << endl;                
                        ck8 << " " << endl; 
                        ck8.close();
      //L'err è la differenza tra il riferimento e la posizione nello spazio operativo misurata

      /*err[0]=ref_pollice_x_max-x_mis_pollice;
      err[1]=ref_pollice_y_max-y_mis_pollice;
      err[2]=ref_pollice_z_max-z_mis_pollice;

      err[3]=ref_indice_x_max-x_mis_indice;
      err[4]=ref_indice_y_max-y_mis_indice;
      err[5]=ref_indice_z_max-z_mis_indice;

      err[6]=ref_medio_x_max-x_mis_medio;
      err[7]=ref_medio_y_max-y_mis_medio;
      err[8]=ref_medio_z_max-z_mis_medio;

      err[9]=ref_anulare_x_max-x_mis_anulare;
      err[10]=ref_anulare_y_max-y_mis_anulare;
      err[11]=ref_anulare_z_max-z_mis_anulare;

      err[12]=ref_mignolo_x_max-x_mis_mignolo;
      err[13]=ref_mignolo_y_max-y_mis_mignolo;
      err[14]=ref_mignolo_z_max-z_mis_mignolo;*/

      err[0]=ref_pollice_x-x_mis_pollice;
      err[1]=ref_pollice_y-y_mis_pollice;
      err[2]=ref_pollice_z-z_mis_pollice;

      err[3]=ref_indice_x-x_mis_indice;
      err[4]=ref_indice_y-y_mis_indice;
      err[5]=ref_indice_z-z_mis_indice;

      err[6]=ref_medio_x-x_mis_medio;
      err[7]=ref_medio_y-y_mis_medio;
      err[8]=ref_medio_z-z_mis_medio;

      err[9]=ref_anulare_x-x_mis_anulare;
      err[10]=ref_anulare_y-y_mis_anulare;
      err[11]=ref_anulare_z-z_mis_anulare;

      err[12]=ref_mignolo_x-x_mis_mignolo;
      err[13]=ref_mignolo_y-y_mis_mignolo;
      err[14]=ref_mignolo_z-z_mis_mignolo;  
        
                        std::ofstream ck4;
                        ck4.open("/home/fanny/catkin_ws/errore.txt",std::ios_base::app);
                        if(!ck4){
                                cout<<"errore nella creazione del file!";
                                return -1;
                                 }

                        ck4 << err << endl;
                        ck4 << " " << endl; 
                        ck4.close();  

      //errore è err dopo la moltiplicazione per il guadagno (controllore)

      errore=Data(500*err[0],
                 500*err[1],
                 500*err[2],
                 500*err[3],
                 500*err[4],
                 500*err[5],
                 500*err[6],
                 500*err[7],
                 500*err[8],
                 500*err[9],
                 500*err[10],
                 500*err[11],
                 500*err[12],
                 500*err[13],
                 500*err[14]  );

          //Jacobiano delle sinergie
          Jac=J*E;
          
          // vettore delle sinergie sigma punto
          dq =Jac.T()*errore; 

          // Integrazione di Eulero -> calcolo sigma 3 x 1
          dq_k_precedente= sigma;
          dq_k=dq_k_precedente+(dq*1/100); 

         // vettore angoli di giunto 1 x 9
         m=dq_k.T()*E.T();   

        /*if (m[0][0]>=-113.20 && m[0][0]<=0)
         m[0][0]=m[0][0];
     else if (m[0][0]<-113.20)
             m[0][0]=-113.20;
         else m[0][0]=0;


     if (m[0][1]>=0 && m[0][1]<=192.65)
         m[0][1]=m[0][1];
     else if (m[0][1]<0)
             m[0][1]=0;
         else m[0][1]=192.65;

     if (m[0][8]>=0 && m[0][8]<=66.08)//;%3
         m[0][8]=m[0][8];
     else if (m[0][8]<0)
             m[0][8]=0;
         else m[0][8]=66.08;


     if (m[0][3]>=0 && m[0][3]<=45.75)
         m[0][3]=m[0][3];
     else if (m[0][3]<0)
             m[0][3]=0;
         else m[0][3]=45.75;

     if (m[0][2]>=0 && m[0][2]<=156.3)
         m[0][2]=m[0][2];
     else if (m[0][2]<0)
             m[0][2]=0;
         else m[0][2]=156.3;

     if (m[0][5]>=0 && m[0][5]<=45.75)
         m[0][5]=m[0][5];
     else if (m[0][5]<0)
             m[0][5]=0;

         else m[0][5]=45.75;


     if (m[0][4]>=0 && m[0][4]<=156.33)
         m[0][4]=m[0][4];
     else if (m[0][4]<0)
             m[0][4]=0;
         else m[0][4]=156.33;

     if (m[0][6]>=0 && m[0][6]<=212.58)
         m[0][6]=m[0][6];
     else if (m[0][6]<0)
             m[0][6]=0;
         else m[0][6]=212.58;

     if (m[0][7]>=0 && m[0][7]<=212.73)
         m[0][7]=m[0][7];
     else if (m[0][7]<0)
             m[0][7]=0;
         else m[0][7]=212.73;*/

      //Pollice
      q1=0.5*(m[0][0]);
      q2=0.29*(m[0][1])+20.7941;
      q3=0.29*(m[0][1])+180-161.8544;
      q4=0.42*(m[0][1])+180-176.4734;

      //indice

      q10=0.49*(m[0][2])+180-161.8584;
      q6a=m[0][3]+23.7532;
      q6b=0.25*(m[0][8]-66.08);
      q14=0.51*(m[0][2])+180-168.8171;

      //medio
      q11=0.49*(m[0][4])+180-161.7631;
      q7=m[0][5]+23.7532;
      q15=0.51*(m[0][4])+180-168.1028;
 
      //opposizione 
      q5=0.5*(m[0][0]);

      //anulare
      q8b=0.25*(m[0][8]-66.08);
      q8a=0.26*(m[0][6])+25.641;
      q12=0.36*(m[0][6])+180-161.7631;
      q16=0.38*(m[0][6])+180-168.1028;

      //mignolo
      q9b=0.5*(m[0][8]-66.08);

      q9a=0.26*(m[0][7])+25.641;
      q13=0.36*(m[0][7])+180-162.0458;
      q17=0.38*(m[0][7])+180-169.6355;

       //Il sw Ros accetta come riferimento da dare alla mano il valore dei seguenti giunti (in radianti)
       q_ros=Data(-q1, q2-20.7941, q10-180+161.8584, q6a-23.7532, q11-180+161.7631, q7-23.7532, q8a-25.6410, q9a-25.6410, -q9b);
         
         for (int i=0;i<8;i++)
         {
                m_rad_ros[0][i]=(PI*q_ros[0][i])/180;
         }
        
         m_rad_riordinato[0][0]=m_rad_ros[0][1];
         m_rad_riordinato[0][1]=m_rad_ros[0][0];
         m_rad_riordinato[0][2]=m_rad_ros[0][2];
         m_rad_riordinato[0][3]=m_rad_ros[0][3];
         m_rad_riordinato[0][4]=m_rad_ros[0][4];
         m_rad_riordinato[0][5]=m_rad_ros[0][5];
         m_rad_riordinato[0][6]=m_rad_ros[0][6];
         m_rad_riordinato[0][7]=m_rad_ros[0][7];
         m_rad_riordinato[0][8]=m_rad_ros[0][8];

                        std::ofstream ck5;
                        ck5.open("/home/fanny/catkin_ws/m_rad_CLIK.txt",std::ios_base::app);
                        if(!f){
                                cout<<"errore nella creazione del file!";
                                return -1;
                                }

                        ck5 << m_rad_riordinato << endl;
                        ck5 << " " << endl;
                        ck5.close();

         if(m_rad_riordinato[0][0]<0)
             {
                m_rad_riordinato[0][0]=0;
             }
        else {
                if(m_rad_riordinato[0][0]>=0.97)
                {
                        m_rad_riordinato[0][0]=0.95;
                }
             }

         if(m_rad_riordinato[0][1]<0)
         {
             m_rad_riordinato[0][1]=0;
         }else
         {
              if(m_rad_riordinato[0][1]>=0.99)
              {
                   m_rad_riordinato[0][1]=0.97;
              }
         }
         
         if(m_rad_riordinato[0][2]<0)
         {
              m_rad_riordinato[0][2]=0;
         }
         else
         {
             if(m_rad_riordinato[0][2]>=1.33)
             {
                  m_rad_riordinato[0][2]=1.3;
             }
         }
         
         if(m_rad_riordinato[0][3]<0)
         {
              m_rad_riordinato[0][3]=0;
         }
         else
         {
             if(m_rad_riordinato[0][3]>=0.8)
             {
                 m_rad_riordinato[0][3]=0.77;
             }
         }
         
         if(m_rad_riordinato[0][4]<0)
         {
               m_rad_riordinato[0][4]=0;
         }
         else 
         {
              if(m_rad_riordinato[0][4]>=1.33)
              {
                  m_rad_riordinato[0][4]=1.3;
              }
         }
         
         if(m_rad_riordinato[0][5]<0)
         {
                 m_rad_riordinato[0][5]=0;
         }else
         {
              if(m_rad_riordinato[0][5]>=0.8)
              {
                  m_rad_riordinato[0][5]=0.77;
              }
         }
         
         if(m_rad_riordinato[0][6]<0)
         {
                 m_rad_riordinato[0][6]=0;
         }
         else
         {
             if(m_rad_riordinato[0][6]>=0.98)
             {
                   m_rad_riordinato[0][6]=0.95;
             }
         }
         
         if(m_rad_riordinato[0][7]<0)
         {
                m_rad_riordinato[0][7]=0;
         }
         else 
         {
              if(m_rad_riordinato[0][7]>=0.98)
              {
                 m_rad_riordinato[0][7]=0.95;
              }
         }
         
         if(m_rad_riordinato[0][8]<0)
         {
               m_rad_riordinato[0][8]=0;
         }
         else 
         {
              if(m_rad_riordinato[0][8]>=0.58)
              {
                    m_rad_riordinato[0][8]=0.55;
              }
         }


             //Controllo sulle soglie di corrente

             if(abs(svh_node.getCurrentPositions().effort[0])>=380) d[0]=0.0;
                         
             if(abs(svh_node.getCurrentPositions().effort[1])>=120) d[1]=0.0; 
             
             if(abs(svh_node.getCurrentPositions().effort[2])>=140) d[2]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[3])>=180) d[3]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[4])>=120) d[4]=0.0;

             if(abs(svh_node.getCurrentPositions().effort[5])>=120) d[5]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[6])>=120) d[6]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[7])>=120) d[7]=0.0;
                                
/*             Vector<9> motori;
             for(size_t channel=0; channel<9;++channel)
             {
                 if (svh_node.getCurrentPositions().position[channel]<m_rad_riordinato[0][channel])
                   {
                 //Microinterpolazione sui valori degli angoli di giunto
                 channel_pos.position[channel]=svh_node.getCurrentPositions().position[channel]+d[channel];
                 motori[channel]=svh_node.getCurrentPositions().position[channel]+d[channel];;
                 channel_cost.position[channel]=svh_node.getCurrentPositions().position[channel]+d[channel];

                 }
             }
             channel_ced_pub.publish(channel_pos);
             channel_cost_pub.publish(channel_cost);*/

             
             for(size_t channel=0; channel<9;++channel)
             {
                       if(d[channel]!= 0)
                       {                        
                                channel_pos.position[channel]=m_rad_riordinato[0][channel];
                       } 

                        std::ofstream ck6;
                        ck6.open("/home/fanny/catkin_ws/m_pubblicati.txt",std::ios_base::app);
                        if(!ck6){
                                cout<<"errore nella creazione del file!";
                                return -1;
                                 }

                        ck6 << channel_pos.position[channel] << " "; 
                        ck6.close();                    
             }

             std::ofstream ck6;
             ck6.open("/home/fanny/catkin_ws/m_pubblicati.txt",std::ios_base::app);
             if(!ck6){
                      cout<<"errore nella creazione del file!";
                      return -1;
                      }

                                
              ck6 << " " << endl;
              ck6.close();
     
             channel_ced_pub.publish(channel_pos);
             
            
    // get the current positions of all joints and publish them

    ros::spinOnce();

    rate.sleep();

          }
}
                  }
/*
      //Assunzione posizione calcolata con la Pca per ciascuna delle prese
      else if(correzione==false && standard==false)// && syn==false)
      {       
         while (nh.ok())
          {
            //Andamento lineare da home a pca
            
             for(size_t channel=0; channel<9;++channel)
             { 
                if(q_pca[channel]>q_home[channel])
                {
                        if(svh_node.getCurrentPositions().position[channel]>=q_pca[channel]) d[channel]=0.0;
                }else 
                {
                        if(svh_node.getCurrentPositions().position[channel]<=q_pca[channel]) d[channel]=0.0;
                }
             }

             //Controllo sulle soglie di corrente
             
             /*if(abs(svh_node.getCurrentPositions().effort[0])>=380) d[0]=0.0;
                         
             if(abs(svh_node.getCurrentPositions().effort[1])>=120) d[1]=0.0; 
             
             if(abs(svh_node.getCurrentPositions().effort[2])>=140) d[2]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[3])>=180) d[3]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[4])>=120) d[4]=0.0;

             if(abs(svh_node.getCurrentPositions().effort[5])>=120) d[5]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[6])>=120) d[6]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[7])>=120) d[7]=0.0;            

            
             if (dita_contatto == 4)
             {
                if(q_pca[7]>q_home[7])
                {
                        if(svh_node.getCurrentPositions().position[7]>=q_pca[7]) d[7]=0.0;
                }else 
                {
                        if(svh_node.getCurrentPositions().position[7]<=q_pca[7]) d[7]=0.0;
                }
             }
                
             if (dita_contatto == 3)
             {
               
               for(int channel=6; channel<8; channel++)
               {  
                 if(q_pca[channel]>q_home[channel])
                 {
                         if(svh_node.getCurrentPositions().position[channel]>=q_pca[channel]) d[channel]=0.0;
                 }else 
                 {
                         if(svh_node.getCurrentPositions().position[channel]<=q_pca[channel]) d[channel]=0.0;
                 }
               }             
              }

             
             if (dita_contatto == 2)
             {
               for(int channel=4; channel<8; channel++)
               {  
                 if(q_pca[channel]>q_home[channel])
                 {
                         if(svh_node.getCurrentPositions().position[channel]>=q_pca[channel]) d[channel]=0.0;
                 }else 
                 {
                         if(svh_node.getCurrentPositions().position[channel]<=q_pca[channel]) d[channel]=0.0;
                 }
               }                 
              }
              
              //channel_ced_pub.publish(channel_pos);

                         for(size_t channel=0; channel<9;++channel)
                         {  
    
                            if(d[channel] == 0.0)
                            {
                                //channel_pos.position[channel]=svh_node.getCurrentPositions().position[channel];
                            }                        
                            else{

                              channel_pos.position[channel]=0.2*((q_pca[channel]-q_home[channel])/abs(q_pca[channel]-q_home[channel]))*count*dt + q_home[channel];
                              
                              if(channel_pos.position[channel]<=0) channel_pos.position[channel]=0;        
                              if(channel_pos.position[channel]>=fine_corsa_max[channel]) channel_pos.position[channel]=fine_corsa_max[channel];

                             //channel_pos.position[channel]=q_pca[channel];   
                             }

                                std::ofstream ck2;
                                ck2.open("/home/fanny/catkin_ws/cp.txt",std::ios_base::app);
                                if(!ck2){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                ck2 << svh_node.getCurrentPositions().position[channel] << " "; 
                                ck2.close();

                                std::ofstream ck10;
                                ck10.open("/home/fanny/catkin_ws/pub.txt",std::ios_base::app);
                                if(!ck10){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                ck10 << channel_pos.position[channel] << " "; 
                                ck10.close();    
                         }

                                std::ofstream ck10;
                                ck10.open("/home/fanny/catkin_ws/pub.txt",std::ios_base::app);
                                if(!ck10){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                
                                ck10 << " " << endl;
                                ck10.close();

                                std::ofstream ck2;
                                ck2.open("/home/fanny/catkin_ws/cp.txt",std::ios_base::app);
                                if(!ck2){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                ck2 << " " << endl; 
                                ck2.close();

                         channel_ced_pub.publish(channel_pos);
                         count = count + 1;

            ros::spinOnce();
            rate.sleep();
          }
      }*/

      //Assunzione posizione calcolata con la Pca per ciascuna delle prese
      else if(correzione==false && standard==false && syn==false)
      {
          while (nh.ok())
          {

            /*if(abs(svh_node.getCurrentPositions().effort[0])>=380) d[0]=0.0;
                         
             if(abs(svh_node.getCurrentPositions().effort[1])>=120) d[1]=0.0; 
             
             if(abs(svh_node.getCurrentPositions().effort[2])>=140) d[2]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[3])>=180) d[3]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[4])>=120) d[4]=0.0;

             if(abs(svh_node.getCurrentPositions().effort[5])>=120) d[5]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[6])>=120) d[6]=0.0;
             
             if(abs(svh_node.getCurrentPositions().effort[7])>=120) d[7]=0.0;  */   
            channel_ced_pub.publish(channel_pos);

                         for(size_t channel=0; channel<9;++channel)
                         {
                             if(d[channel]!=0)
                             {
                                channel_pos.position[channel]=q_pca[channel];
                                std::ofstream ck7;
                                ck7.open("/home/fanny/catkin_ws/cp.txt",std::ios_base::app);
                                if(!ck7){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                ck7 << svh_node.getCurrentPositions().position[channel] << " "; 
                                ck7.close();
                             }else 
                                {
                                     channel_pos.position[channel]=svh_node.getCurrentPositions().position[channel];
                                }
                        }                                
                                std::ofstream ck7;
                                ck7.open("/home/fanny/catkin_ws/cp.txt",std::ios_base::app);
                                if(!ck7){
                                        cout<<"errore nella creazione del file!";
                                        return -1;
                                       }

                                ck7 << " " << endl; 
                                ck7.close();

                         channel_ced_pub.publish(channel_pos);
                

            ros::spinOnce();
            rate.sleep();
          }
      }

              else if(standard==true && syn==false)
               {

                  while (nh.ok())
                  {
                    // get the current positions of all joints and publish them
                    channel_pos_pub.publish(svh_node.getCurrentPositions());

                    ros::spinOnce();
                    rate.sleep();
                  }
               }



               else if(syn==true)
              {
                  //float t=0;
               //const double loop_rate = 50;
                // Time of a half Sin. i.e. 10 = In 10 Seconds the selected fingers will perform a close and open (Sin to 1PI)
                const double sin_duration = 4;
                int sinergia=3;
                Vector<9> E_syn;
                E_syn=Data(E[0][sinergia-1],
                           E[1][sinergia-1],
                           E[2][sinergia-1],
                           E[3][sinergia-1],
                           E[4][sinergia-1],
                           E[5][sinergia-1],
                           E[6][sinergia-1],
                           E[7][sinergia-1],
                           E[8][sinergia-1]
                                
                                );

                Vector<9> mean_value;
                ros::Time counter = ros::Time::now();
                double normalized_time = 0;
                mean_value=Data(0.2109, 0.4064,    0.3020,    0.5998,    0.3338,    0.6404,    0.4899,    0.4960,    0.5384);
                while (nh.ok())
                 {
               // Set up the normalized time

                
                // Init the Random
                srand (time(NULL));

                  if ((ros::Time::now() - counter) > ros::Duration(sin_duration))
                    {
                     counter = ros::Time::now();
                    }
                  else
                    {
                    normalized_time = (ros::Time::now() - counter).toSec() / sin_duration;
                    }

                    for (size_t channel = 0; channel < 9; ++channel)
                    {

                        double cur_pos = 0.0;

                        channel_pos.position[channel] = cur_pos;
                    }
                 
                    // Calculate a halfe sin for the fingers
                    double cur_pos = 5*sin(normalized_time*3.14*2); //per configurazioni min e max impostare questo solo a +-8
                   
                    channel_pos.position[0] = E_syn[0]*cur_pos + mean_value[0];
                    channel_pos.position[1] = E_syn[1]*cur_pos + mean_value[1];
                    channel_pos.position[2] = E_syn[2]*cur_pos + mean_value[2];
                    channel_pos.position[3] = E_syn[3]*cur_pos + mean_value[3];
                    channel_pos.position[4] = E_syn[4]*cur_pos + mean_value[4];
                    channel_pos.position[5] = E_syn[5]*cur_pos + mean_value[5];
                    channel_pos.position[6] = E_syn[6]*cur_pos + mean_value[6];
                    channel_pos.position[7] = E_syn[7]*cur_pos + mean_value[7];
                    channel_pos.position[8] = E_syn[8]*cur_pos + mean_value[8];

                    if(channel_pos.position[0]>=0.93)
                    channel_pos.position[0]=0.93;
                    if(channel_pos.position[1]>=0.95)
                        channel_pos.position[1]=0.95;
                    if(channel_pos.position[2]>=1.3)
                        channel_pos.position[2]=1.3;
                    if(channel_pos.position[3]>=0.76)
                        channel_pos.position[3]=0.76;
                    if(channel_pos.position[4]>=1.29)
                        channel_pos.position[4]=1.29;
                    if(channel_pos.position[5]>=0.76)
                        channel_pos.position[5]=0.76;
                    if(channel_pos.position[6]>=0.94)
                       channel_pos.position[6]=0.94;
                    if(channel_pos.position[7]>=0.94)
                        channel_pos.position[7]=0.94;
                    if(channel_pos.position[8]>=0.56)
                        channel_pos.position[8]=0.56;

                    if(channel_pos.position[0]<0.05)
                    channel_pos.position[0]=0.05;
                    if(channel_pos.position[1]<0)
                        channel_pos.position[1]=0;
                    if(channel_pos.position[2]<0)
                        channel_pos.position[2]=0;
                    if(channel_pos.position[3]<0)
                        channel_pos.position[3]=0;
                    if(channel_pos.position[4]<0)
                        channel_pos.position[4]=0;
                    if(channel_pos.position[5]<0)
                        channel_pos.position[5]=0;
                    if(channel_pos.position[6]<0)
                       channel_pos.position[6]=0;
                    if(channel_pos.position[7]<=0)
                        channel_pos.position[7]=0;
                    if(channel_pos.position[8]<=0)
                        channel_pos.position[8]=0;


                    // Publish
                    channel_ced_pub.publish(channel_pos);

                    rate.sleep();
                    ros::spinOnce();
                   
                    }
                  }




  return 0;
          }

