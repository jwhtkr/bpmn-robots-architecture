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

  std::string service_provided_topic,
              ros_node_server_start_topic,
              ros_node_server_kill_topic,
              config_file,
              missing_params;

  if(!p_nh.getParam("service_provided_topic", service_provided_topic))
  {
    missing_params += "\tservice_provided_topic\n";
  }
  if(!p_nh.getParam("ros_node_server_start_topic", ros_node_server_start_topic))
  {
    missing_params += "\tros_node_server_start_topic\n";
  }
  if(!p_nh.getParam("ros_node_server_kill_topic", ros_node_server_kill_topic))
  {
    missing_params += "\tros_node_server_kill_topic\n";
  }
  if(!p_nh.getParam("config_file", config_file))
  {
    missing_params += "\tconfig_file\n";
  }

  if(!missing_params.empty())
  {
    ROS_ERROR_STREAM("The node " + ros::this_node::getName() + " can't find the following" +
                     " ROS Server Parameters:\n" + missing_params);
    ROS_ERROR_STREAM("The node " + ros::this_node::getName() + " is exiting");
    exit(EXIT_FAILURE);
  }

  thread_pool::ThreadPool masterThreadPool(service_provided_topic,
                                           ros_node_server_start_topic,
                                           ros_node_server_kill_topic,
                                           config_file);

  ros::Rate loop_rate(30);

  while(m_nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}

/* thread_pool_node.cpp */
