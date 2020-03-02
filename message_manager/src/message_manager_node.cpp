/**
 * @File: message_manager_node.cpp
 * @Date: 19 August 2019
 * @Author: James Swedeen
 *
 * @brief
 * A node that can be used to pass messages between BPMN message elements.
 **/

/* Local Headers */
#include"message_manager/message_manager.hpp"

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_manager");
  ros::NodeHandle m_nh;
  ros::NodeHandle p_nh("~");

  std::string new_task_topic, config_file_path;

  ros::Rate loop_rate(50);

  if(!p_nh.getParam("new_task_topic",  new_task_topic) ||
     !p_nh.getParam("config_file_path", config_file_path))
  {
    throw std::runtime_error("ROS parameters for this node ain't set");
  }

  MessageManager message_manager("http://localhost:8080/",
                                 "Message_Manager",
                                 new_task_topic,
                                 config_file_path);

  while(m_nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}

/* message_manager_node.cpp */
