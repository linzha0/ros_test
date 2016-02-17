/* Head file for example of algorithm structure */
/* Author :Lin Zhao */
/***********************************************************/

#ifndef MY_NODELET_MY_NODELET_HEADFILE
#define MY_NODELET_MY_NODELET_HEADFILE

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"
#include <boost/thread.hpp>
#include "nodelet/nodelet.h"
#include "message_filters/subscriber.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/PointCloud2.h"

namespace my_nodelet
{
  using namespace message_filters::sync_policies;

  class MyNodelet : public nodelet:: Nodelet
	{
	  public: 
		MyNodelet();

	  private: 	

		//Funcations 
		virtual void onInit();

		void connectCb();

		void dataCb(const sensor_msgs::PointCloud2ConstPtr& A_msg,
					const sensor_msgs::PointCloud2ConstPtr& B_msg,
					const sensor_msgs::PointCloud2ConstPtr& C_msg);
		
		// Subscriptions
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_A;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_B;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_C;
    
        typedef ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
        typedef ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSyncPolicy;
        typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
        typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
        boost::shared_ptr<Synchronizer> sync_;
        boost::shared_ptr<ExactSynchronizer> exact_sync_;
    
        // Publications
        boost::mutex connect_mutex_;
        ros::Publisher pub_D;
		ros::NodeHandle nh_, private_nh_;

		// ROS Parameters
		unsigned int queue_size;
		double para_;


	};

}//my_nodelet

#endif //MY_NODELET_MY_NODELET_HEADFILE
