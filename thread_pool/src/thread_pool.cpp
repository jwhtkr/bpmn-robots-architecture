/**
 * @File: thread_pool.cpp
 * @Date: March 2020
 * @Author: James Swedeen
 **/

/* Local Headers */
#include"thread_pool/thread_pool.hpp"
#include"thread_pool/data_objects.hpp"

/* ROS Node Server Headers */
#include<launcher_helpers/param_server_phraser.hpp>

/* ROS Node Server Messages */
#include<node_server_msgs/StartNodeList.h>
#include<node_server_msgs/Kill.h>

#include<node_server_msgs/Target.h>

/* Architecture Messages */
#include<architecture_msgs/ModifyRobots.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<set>
#include<mutex>
#include<memory>
#include<stdexcept>
#include<vector>
#include<utility>
#include<system_error>
#include<fstream>

namespace thread_pool
{
  ThreadPool::ThreadPool(const std::string& service_provided_topic,
                         const std::string& ros_node_server_start_topic,
                         const std::string& ros_node_server_kill_topic,
                         const std::string& config_namespace)
   : ros_node_server_start_regex(ros_node_server_start_topic),
     ros_node_server_kill_regex( ros_node_server_kill_topic),
     resources(parseConfig(config_namespace)),
     provide_srv(c_nh.advertiseService(service_provided_topic, &ThreadPool::setupRobotsCallback, this))
  {}

  ThreadPool::~ThreadPool() noexcept
  {
    this->provide_srv.shutdown();
    this->srv_mux.lock();
  }

  std::shared_ptr<ThreadPool> ThreadPool::makeThreadPool(const ros::NodeHandle& p_nh)
  {
    std::string service_provided_topic,
                ros_node_server_start_topic,
                ros_node_server_kill_topic,
                config_namespace,
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
    if(!p_nh.getParam("config_namespace", config_namespace))
    {
      missing_params += "\tconfig_namespace\n";
    }

    if(!missing_params.empty())
    {
      ROS_ERROR_STREAM("ThreadPool::makeThreadPool error, the namespace " + p_nh.getNamespace() +
                       " Doesn't hold the following ROS Server Parameters:\n" + missing_params);
    }

    return std::make_shared<thread_pool::ThreadPool>(service_provided_topic,
                                                     ros_node_server_start_topic,
                                                     ros_node_server_kill_topic,
                                                     config_namespace);
  }

  bool ThreadPool::setupRobotsCallback(architecture_msgs::ModifyRobots::Request& req,
                                       architecture_msgs::ModifyRobots::Response&)
  {
    try
    {
      std::lock_guard<std::mutex> srv_lock(this->srv_mux);

      for(auto robot_it = req.robots.cbegin(); robot_it != req.robots.cend(); robot_it++)
      {
        RobotServices& robot_srv = this->findNodeServer(robot_it->name);

        // End the nodes requested
        for(auto resources_it = robot_it->end.cbegin(); resources_it != robot_it->end.cend(); resources_it++)
        {
          node_server_msgs::Kill kill_srv;
          // Find the node
          std::set<ResourceMsgs>::const_iterator m_resource = this->resources.find(ResourceMsgs(*resources_it));
          if(this->resources.end() == m_resource)
          {
            throw std::runtime_error("resource " + *resources_it + " can't be found");
          }

          std::map<std::string, std::vector<node_server_msgs::Target>>::const_iterator m_targets = m_resource->targets.find(robot_it->name);
          if(m_resource->targets.end() == m_targets)
          {
            throw std::runtime_error("targets for resource '" + *resources_it + "' and robot '" + robot_it->name + "' could not be found");
          }

          // End the node
          kill_srv.request.targets = std::move(m_targets->second);

          if(!robot_srv.ender.call(kill_srv))
          {
            throw std::runtime_error("ROS Node Server failed to kill one or all nodes requested");
          }

          m_resource->targets.erase(m_targets);
        }

        // Start the nodes requested
        for(auto resources_it = robot_it->start.cbegin(); resources_it != robot_it->start.cend(); resources_it++)
        {
          node_server_msgs::StartNodeList start_srv;
          node_server_msgs::Target        targets;
          // Find message data
          std::set<ResourceMsgs>::iterator m_resource = this->resources.find(ResourceMsgs(*resources_it));
          if(this->resources.end() == m_resource)
          {
            throw std::runtime_error("can't find startup information for resource " + *resources_it);
          }

          // Setup service call
          start_srv.request.name  = robot_it->name + std::string("/") + *resources_it;
          start_srv.request.nodes = m_resource->start;
          for(auto service_it = start_srv.request.nodes.begin(); service_it != start_srv.request.nodes.end(); service_it++)
          {
            if(std::string() != service_it->ns)
            {
              service_it->ns = robot_it->name + std::string("/") + service_it->ns;
            }
            else
            {
              service_it->ns = robot_it->name;
            }
          }

          // Ask for nodes to be started
          robot_srv.starter.call(start_srv);

          // Make sure it worked
          for(auto started_it = start_srv.response.results.begin(); started_it != start_srv.response.results.end(); started_it++)
          {
            if(std::string() == started_it->name)
            {
              throw std::runtime_error("the ROS Node Server failed to launch one or all of the resources requested");
            }
          }

          // Fill in target information for those nodes
          std::vector<node_server_msgs::Target>& target_ref = m_resource->targets[robot_it->name];

          target_ref.emplace_back();

          target_ref.back().type = node_server_msgs::Target::GROUP;
          target_ref.back().name = robot_it->name + "/" + *resources_it;
        }
      }

      return true;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ThreadPool::setupRobotsCallback error, " + static_cast<std::string>(e.what()));
    }
  }

  RobotServices& ThreadPool::findNodeServer(const std::string& name)
  {
    // Look for the right robot service clients
    std::set<RobotServices>::const_iterator service = this->robots_srv.find(RobotServices(name));

    // If robot isn't present look for it
    if(this->robots_srv.end() == service)
    {
      RobotServices new_srv(name);

      if(!ros::service::exists("/" + name + "/" + this->ros_node_server_start_regex, false))
      {
        throw std::runtime_error("robot '" + name + "' start service can't be found");
      }
      if(!ros::service::exists("/" + name + "/" + this->ros_node_server_kill_regex,  false))
      {
        throw std::runtime_error("robot '" + name + "' kill service can't be found");
      }

      // Add new robot service to this object
      new_srv.starter = this->c_nh.serviceClient<node_server_msgs::StartNodeList>(std::string("/") + name + std::string("/") + this->ros_node_server_start_regex);
      new_srv.ender   = this->c_nh.serviceClient<node_server_msgs::Kill>         (std::string("/") + name + std::string("/") + this->ros_node_server_kill_regex);

      service = this->robots_srv.emplace(std::move(new_srv)).first;
    }

    return const_cast<RobotServices&>(*service);
  }

  std::set<ResourceMsgs> ThreadPool::parseConfig(const std::string config_namespace)
  {
    try
    {
      std::set<ResourceMsgs>          output;
      node_server::ParamServerPhraser phraser;
      ros::NodeHandle                 nh(config_namespace);
      std::vector<std::string>        resource_list;

      // Setup substitution arguments
      if(nh.hasParam("arguments"))
      {
        phraser.configArgs(ros::NodeHandle(config_namespace + "arguments"),    phraser.getGlobalStack());
      }

      if(nh.hasParam("parameters"))
      {
        phraser.uploadParams(ros::NodeHandle(config_namespace), phraser.getGlobalStack());
      }

      // Transform data
      if(nh.getParam("resources", resource_list))
      {
        for(auto name_it = resource_list.begin(); name_it != resource_list.end(); name_it++)
        {
          ResourceMsgs                                      new_start_list(*name_it);
          std::pair<std::set<ResourceMsgs>::iterator, bool> new_resource;

          std::vector<node_server_msgs::NodeStart::Ptr> from_phraser =
            phraser.substituteNodeStartList(ros::NodeHandle(config_namespace, *name_it), phraser.getGlobalStack());
ROS_ERROR_STREAM(**from_phraser.begin());
          new_start_list.start.reserve(from_phraser.size());
          std::for_each(from_phraser.begin(), from_phraser.end(),
            [&new_start_list](const node_server_msgs::NodeStart::Ptr& ittr)
            {
              new_start_list.start.emplace_back(std::move(*ittr));
            });

          new_resource = output.emplace(new_start_list);
          if(!new_resource.second)
          {
            throw std::runtime_error("the resource name " + *name_it + " is repeated which is not aloud.");
          }
        }
      }

      return output;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ThreadPool::parseConfigFile error, " + static_cast<std::string>(e.what()));
    }
  }
}// thread_pool

/* thread_pool.cpp */
