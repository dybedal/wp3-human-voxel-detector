#include <time.h>
#include <stdio.h>
#include <signal.h>

#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace message_filters;

typedef pcl::PointXYZI PCL_PointType;
typedef pcl::PointCloud<PCL_PointType> PointCloudType;
typedef sync_policies::ApproximateTime<PointCloudType, PointCloudType, PointCloudType, PointCloudType> MySyncPolicy;


// Setup
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

class PointCloudMerger
{
public:
	PointCloudMerger() : 
		nh{"~"},
		pc1_sub{nh, "pointcloud1_in", 10},
		pc2_sub{nh, "pointcloud2_in", 10},
		pc3_sub{nh, "pointcloud3_in", 10},
		pc4_sub{nh, "pointcloud4_in", 10},
		sync{MySyncPolicy(100), pc1_sub, pc2_sub, pc3_sub, pc4_sub}

	{
		pub = nh.advertise<PointCloudType>("pointcloud_out", 1);
	
		sync.registerCallback(boost::bind(&PointCloudMerger::callback, this, _1, _2, _3, _4));
	}

	void callback(const PointCloudType::ConstPtr & cloud1, const PointCloudType::ConstPtr & cloud2, const PointCloudType::ConstPtr & cloud3,
		const PointCloudType::ConstPtr & cloud4)
	{
	
		PointCloudType in_cloud;
		in_cloud.width = cloud1->width;
		in_cloud.height = cloud1->height;
		in_cloud.header.frame_id = cloud1->header.frame_id;
		
		in_cloud = *cloud1;
		in_cloud += *cloud2;
		in_cloud += *cloud3;
		in_cloud += *cloud4;

		PointCloudType::Ptr cloud_grid (new PointCloudType());

		pcl::VoxelGrid<PCL_PointType> grid;
		grid.setInputCloud (in_cloud.makeShared());
  		grid.setLeafSize (0.04f, 0.04f, 0.04f);
  		grid.filter (*cloud_grid);
  		

  		PointCloudType::Ptr cloud_filtered (new PointCloudType());

  		pcl::StatisticalOutlierRemoval<PCL_PointType> sor;
  		sor.setInputCloud (cloud_grid);
  		sor.setMeanK (15);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);


		pcl_conversions::toPCL(ros::Time::now(), cloud_filtered->header.stamp);
  		cloud_filtered->header.frame_id = in_cloud.header.frame_id;
		pub.publish(*cloud_filtered);
	}

	void shutdown()
	{
		nh.shutdown();
	}


private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	
	message_filters::Subscriber<PointCloudType> pc1_sub;
	message_filters::Subscriber<PointCloudType> pc2_sub;
	message_filters::Subscriber<PointCloudType> pc3_sub;
	message_filters::Subscriber<PointCloudType> pc4_sub;

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync;
	
};



int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "wp3_decomp_multi", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	

	signal(SIGINT, killHandler);
	signal(SIGTERM, killHandler);


	PointCloudMerger merger;

	ros::Rate loopRate(ROS_RATE);

	

	while(ros::ok()){
		ros::spinOnce();
		loopRate.sleep();
	}
 

	if(ros::ok()){
		merger.shutdown();
		ros::shutdown();
	}


 	ROS_INFO("%s","Shutdown complete.");
	return 0;
}
