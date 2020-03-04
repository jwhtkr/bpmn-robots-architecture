/**
 * @File: message_manager.hpp
 * @Date: 19 August 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class that can be used to pass messages between BPMN message elements.
 **/

#ifndef MESSAGE_MANAGER_MESSAGE_MANAGER_HPP
#define MESSAGE_MANAGER_MESSAGE_MANAGER_HPP

/* BPMN Interface Headers */
#include<bpmn_interface/bpmn_interface.hpp>

/* Architecture Messages */
#include<architecture_msgs/Behavior.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<set>

class MessageManager
{
public:
  /**
   * @Default Constructor
   **/
  MessageManager() = delete;
  /**
   * @Copy Constructor
   **/
  MessageManager(const MessageManager&) = delete;
  /**
   * @Move Constructor
   **/
  MessageManager(MessageManager&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Started listening for new tasks and parses the passed in config list to find out
   * what task names it is responsible for.
   *
   * @parameters
   * camunda_base_uri: The base URI that the camunda server can be found at
   * camunda_worker_id: The ID that Camunda will recognize this node by
   * new_take_topic: The topic that the Behavior Listener is publishing new Camunda
   *                 tasks on
   * config_namespace: The ROS Namespace containing a list of behavior
   *                   names that this node will respond to
   **/
  MessageManager(const std::string& camunda_base_uri,
                 const std::string& camunda_worker_id,
                 const std::string& new_tasks_topic,
                 const std::string& config_namespace);
  /**
   * @Deconstructor
   **/
  ~MessageManager() noexcept = default;
  /**
   * @Assignment Operators
   **/
  MessageManager& operator=(const MessageManager&)  = delete;
  MessageManager& operator=(      MessageManager&&) = delete;
private:
  /* General helper members */
  ros::NodeHandle c_nh;
  /* Listens for new messages to send */
  ros::Subscriber new_task_sub;
  /* Gets and send messages to and from Camunda */
  bpmn::MessageHandler message_handler;
  /* What task names this object is responsible for */
  std::set<std::string> m_tasks;
  /**
   * @newTaskCallback
   *
   * @brief
   * This is the function that will be called whenever the Behavior Listener
   * publishes a task. This function will asses whether or not it is a
   * new message behavior and if it is it will lock it and publish it's variables
   * to were they need to go.
   **/
  void newTaskCallback(const architecture_msgs::Behavior& msg);
  /**
   * @relayMessage
   *
   * @brief
   * This function locks a task on the given camunda topic and publishes that task's
   * variables to wherever is specified by the activity Id of the locked task.
   **/
  void relayMessage(const std::string& camunda_topic);
  /**
   * @needsAction
   *
   * @brief
   * Returns true if the passed in message describes a new message task and false
   * otherwise.
   **/
  bool needsAction(const architecture_msgs::Behavior& msg) const noexcept;
};

#endif
/* message_manager.hpp */
