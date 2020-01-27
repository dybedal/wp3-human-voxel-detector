#include <time.h>
#include <stdio.h>
#include <signal.h>

#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "wp3_decompressor/decompressor.h"

// Setup
#define MIN_POINTS 3 //minimum number of points in compressed voxels
#define ROS_RATE 60

void killHandler(int)
{
  ROS_INFO("%s","Shutdown request received.");
  ros::NodeHandle nh;
  ROS_INFO("%s","Terminating nodehandle.");
  nh.shutdown();
  ROS_INFO("%s","Terminating rosnode.");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "wp3_decompressor", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  signal(SIGINT, killHandler);
  signal(SIGTERM, killHandler);

  ros::Rate loopRate(ROS_RATE);

  // Multithreaded spinner
  //    	ros::AsyncSpinner spinner(8);
  //    	spinner.start();
  //    	usleep(1000000); // give the spinner some time to start (1000 ms)

  if(!nh.hasParam("sensor_name"))
    ROS_ERROR("%s","Missing _sensor_name:=<name> parameter! Shutting down...");
  else if(!nh.hasParam("filter_value"))
    ROS_ERROR("%s","Missing _filter_value:=<intensity> parameter! Shutting down...");
  else if(!nh.hasParam("sensor_type"))
    ROS_ERROR("%s","Missing _sensor_type:=<0: Kinect, 1: Velodyne> parameter! Shutting down...");
  else{

    std::string outputTopic;
    std::string inputTopic;
    std::string sensorName;
    std::string sensorFrame;
    float filterValue;
    int sensorType;


    nh.getParam("sensor_name", sensorName);
    nh.getParam("filter_value", filterValue);
    nh.getParam("sensor_type", sensorType);
    nh.param<std::string>("sensor_frame", sensorFrame, "world");

    switch(sensorType) {
    case 0: // Kinect

      ROS_INFO("Starting node using KINECT V2");
      outputTopic = "/master/"+sensorName+"/kinect_decomp";
      inputTopic = "/"+sensorName+"/wp3/kinect_comp";
      break;

    case 1: // Velodyne
      ROS_INFO("Starting node using VELODYNE");
      outputTopic = "/master/"+sensorName+"/velodyne_decomp";
      inputTopic = "/"+sensorName+"/wp3/velodyne_comp";
      break;

    } // End switch (sensorType)

    ros::Duration(1.0).sleep();

    // Start decompressor
    wp3::CloudDecompressor decompressor(outputTopic, inputTopic, sensorFrame, filterValue, false);


    while(ros::ok()){
      ros::spinOnce();
      loopRate.sleep();
    }
  }

  if(ros::ok()){
    nh.shutdown();
    ros::shutdown();
  }


  ROS_INFO("%s","Shutdown complete.");
  return 0;
}
