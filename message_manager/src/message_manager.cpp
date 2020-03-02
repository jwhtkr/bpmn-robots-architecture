/**
 * @File: message_manager.cpp
 * @Date: 19 August 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class that can be used to pass messages between BPMN message elements.
 **/

/* Local Headers */
#include"message_manager/message_manager.hpp"

/* ROS Node Server Headers */
#include<launcher_helpers/json_phraser.hpp>

/* BPMN Interface Headers */
#include<bpmn_interface/bpmn_interface.hpp>

/* Camunda API Headers */
#include<camunda_c_api/camunda_c_api.hpp>

/* Architecture Messages */
#include<architecture_msgs/Behavior.h>

/* Rest Headers */
#include<cpprest/json.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<stdexcept>
#include<utility>

MessageManager::MessageManager(const std::string& camunda_base_uri,
                               const std::string& camunda_worker_id,
                               const std::string& new_tasks_topic,
                               const std::string& config_file_path)
: new_task_sub(c_nh.subscribe(new_tasks_topic, 100, &MessageManager::newTaskCallback, this)),
  message_handler(camunda_base_uri, camunda_worker_id)
{
  try
  {
    web::json::array config_json(std::move(node_server::JsonPhraser::getJson(config_file_path).as_array()));

    this->m_tasks.reserve(config_json.size());
    for(auto topic_it = config_json.begin(); topic_it != config_json.end(); topic_it++)
    {
      this->m_tasks.emplace_back(std::move(topic_it->as_string()));
    }
  }
  catch(const std::exception& ex)
  {
    throw std::runtime_error("MessageManager::Constructor error, " + static_cast<std::string>(ex.what()));
  }
}

void MessageManager::newTaskCallback(const architecture_msgs::Behavior& msg)
{
  if(this->needsAction(msg))
  {
    this->relayMessage(msg.topic);
  }
}

void MessageManager::relayMessage(const std::string& camunda_topic)
{
  try
  {
    bpmn::TaskLock<> task_lock(this->message_handler.getBaseUri(), this->message_handler.getWorkerId(), camunda::Topics(camunda_topic, 999));

    this->message_handler.sendMessage<>(task_lock.getActivityId(), camunda::Variables(task_lock.getResponsVars()));
  }
  catch(const std::exception& ex)
  {
    throw std::runtime_error("MessageManager::relayMessage error, " + static_cast<std::string>(ex.what()));
  }
}

bool MessageManager::needsAction(const architecture_msgs::Behavior& msg) const noexcept
{
  if(architecture_msgs::Behavior::NEW != msg.status)
  {
    return false;
  }

  for(auto topic_it = this->m_tasks.cbegin(); topic_it != this->m_tasks.cend(); topic_it++)
  {
    if(msg.topic == *topic_it)
    {
      return true;
    }
  }

  return false;
}

/* message_manager.cpp */
