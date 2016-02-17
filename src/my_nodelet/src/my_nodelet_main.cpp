/* main cpp file for algorithm sturecture 
 * Author : Lin Zhao
 ********************************************************/

#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <my_nodelet/my_nodelet_headfile.h>
#include <boost/thread.hpp>
#include <boost/version.hpp>
#include "sensor_msgs/PointCloud2.h"

namespace my_nodelet
{
  MyNodelet::MyNodelet() {}

  void MyNodelet::onInit()
  {
												//boost::mutex::scoped_lock lock(connect_mutex_);
	private_nh_ = getPrivateNodeHandle();
	
//Read pararmeter and check thread
	int concurrency_level;
	bool use_exact_sync;

	private_nh_.param<double>("para", para_,0.0);
	private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param("exact_sync", use_exact_sync, false);

    if (concurrency_level == 1) //check if single threaded, otherwise let nodelet manager dictate thread pool size
    {
      nh_ = getNodeHandle();
    }
    else
    {
      nh_ = getMTNodeHandle();
    }

    if (concurrency_level > 0) //Only queue one data per running thread
    {
      queue_size = concurrency_level;
    }
    else
    {
      queue_size= boost::thread::hardware_concurrency();
    }

// Synchronize inputs.
    if (use_exact_sync)
    {
      exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_A, sub_B, sub_C ));
      exact_sync_->registerCallback(boost::bind(&MyNodelet::dataCb, this, _1, _2, _3));
    }
    else
    {
      sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_A, sub_B, sub_C ));
      sync_->registerCallback(boost::bind(&MyNodelet::dataCb, this, _1, _2, _3));
    }

// Monitor whether anyone is subscribed to the output
    ros::SubscriberStatusCallback connect_cb = boost::bind(&MyNodelet::connectCb, this);
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
  
    pub_D = nh_.advertise<std_msgs::Float64>("data",1, connect_cb , connect_cb );

  }

  void MyNodelet::connectCb()
  {
	boost::lock_guard<boost::mutex> lock(connect_mutex_);
	if (pub_D.getNumSubscribers() == 0)
	{
	  sub_A.unsubscribe();
	  sub_B.unsubscribe();
	  sub_C.unsubscribe();
	}
	else if (pub_D.getNumSubscribers() > 0 && sub_A.getSubscriber().getNumPublishers() == 0)
	{
	  sub_A.subscribe(nh_, "Topic_A", queue_size);
	  sub_B.subscribe(nh_, "Topic_B", queue_size);
	  sub_C.subscribe(nh_, "Topic_C", queue_size);
	}
  }

  void MyNodelet::dataCb (const sensor_msgs::PointCloud2ConstPtr& A_msg,
			    		  const sensor_msgs::PointCloud2ConstPtr& B_msg,
			   			  const sensor_msgs::PointCloud2ConstPtr& C_msg)
  {
	std_msgs::Float64 data_msg;
	data_msg.data = 1;
	pub_D.publish(data_msg);
	
  }


}







#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_nodelet::MyNodelet, nodelet::Nodelet);

