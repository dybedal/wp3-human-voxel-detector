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

using namespace message_filters;

typedef pcl::PointXYZI PCL_PointType;
typedef pcl::PointCloud<PCL_PointType> PointCloudXYZI;
typedef sync_policies::ApproximateTime<PointCloudXYZI, PointCloudXYZI, PointCloudXYZI, PointCloudXYZI, PointCloudXYZI, PointCloudXYZI> MySyncPolicy;



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
		pc5_sub{nh, "pointcloud5_in", 10},
		pc6_sub{nh, "pointcloud6_in", 10},
		sync{MySyncPolicy(100), pc1_sub, pc2_sub, pc3_sub, pc4_sub, pc5_sub, pc6_sub}

	{
		pub = nh.advertise<PointCloudXYZI>("pointcloud_out", 1);
	
		sync.registerCallback(boost::bind(&PointCloudMerger::callback, this, _1, _2, _3, _4, _5, _6));
	}

	void callback(const PointCloudXYZI::ConstPtr & cloud1, const PointCloudXYZI::ConstPtr & cloud2, const PointCloudXYZI::ConstPtr & cloud3,
		const PointCloudXYZI::ConstPtr & cloud4, const PointCloudXYZI::ConstPtr & cloud5, const PointCloudXYZI::ConstPtr & cloud6)
	{
	
		PointCloudXYZI out_cloud;
		out_cloud.width = cloud1->width;
		out_cloud.height = cloud1->height;
		out_cloud.header.frame_id = cloud1->header.frame_id;
		
		out_cloud = *cloud1;
		out_cloud += *cloud2;
		out_cloud += *cloud3;
		out_cloud += *cloud4;
		out_cloud += *cloud5;
		out_cloud += *cloud6;

		pub.publish(out_cloud);
	}

	void shutdown()
	{
		nh.shutdown();
	}


private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	
	message_filters::Subscriber<PointCloudXYZI> pc1_sub;
	message_filters::Subscriber<PointCloudXYZI> pc2_sub;
	message_filters::Subscriber<PointCloudXYZI> pc3_sub;
	message_filters::Subscriber<PointCloudXYZI> pc4_sub;
	message_filters::Subscriber<PointCloudXYZI> pc5_sub;
	message_filters::Subscriber<PointCloudXYZI> pc6_sub;

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
