/**
 * @File: thread_pool_node.cpp
 * @Date: 23 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Node used to control any number of ROS Node Server.
 **/

/* Local Headers */
#include"thread_pool/thread_pool.hpp"

/* ROS Headers */
#include<ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_pool");
  ros::NodeHandle m_nh;
  ros::NodeHandle p_nh("~");

  std::shared_ptr<thread_pool::ThreadPool> masterThreadPool(thread_pool::ThreadPool::makeThreadPool(p_nh));

  ros::Rate loop_rate(p_nh.param("spin_rate", 30));

  while(m_nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}

/* thread_pool_node.cpp */
