/*
 * decompressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "wp3_decompressor/decompressor.h"

namespace wp3 {

// Constructor
CloudDecompressor::CloudDecompressor(std::string outputCloudTopic, std::string inputMsgTopic, std::string sensorFrame, const float intensityLimit, const bool showStatistics) :
  decompressedCloud(new PointCloudXYZI ()),
  outputCloud(new PointCloudXYZI ()),
  ptfilter(false),
  sensorFrame(sensorFrame),
  intensityLimit(intensityLimit),
  showStatistics(showStatistics),
  logFile("/home/sfi/decompressorlog.txt"),
  pointCloudDecoder(showStatistics)
{
  if(showStatistics){
    logStream.open(logFile.c_str());
    logStream << "Time" << std::endl;
  }

  sub = nh.subscribe<std_msgs::String>(inputMsgTopic, 1, &wp3::CloudDecompressor::roscallback, this);
  pub = nh.advertise<PointCloudXYZI>(outputCloudTopic, 1);
}

CloudDecompressor::~CloudDecompressor(){
  if(showStatistics)
    logStream.close();
}

// Callback for ROS subscriber
void CloudDecompressor::roscallback(const std_msgs::String::ConstPtr & msg){

  time_t start = clock();

  // Stream for storing serialized compressed point cloud
  std::stringstream compressedData;

  // Retreive data from message
  compressedData << msg->data;

  // Decode stream to point cloud
  pointCloudDecoder.decodePointCloud (compressedData, decompressedCloud);

  // Filter point cloud based on intensity
  ptfilter.setInputCloud (decompressedCloud);
  ptfilter.setFilterFieldName ("intensity");
  ptfilter.setFilterLimits (intensityLimit, FLT_MAX);
  ptfilter.filter (*outputCloud);

  clock_t end = clock();
  double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;

  // Publish the decompressed cloud
  outputCloud->header.frame_id = sensorFrame;
  pub.publish(outputCloud);

  if (showStatistics)
  {
    logStream << time << std::endl;
  }

}

}
