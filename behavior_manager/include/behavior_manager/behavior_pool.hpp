/**
 * @File: behavior_pool.hpp
 * @Date: 23 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This class is used to manage instances of any child of the BehaviorManager class.
 **/

#ifndef BEHAVIOR_POOL_BEHAVIOR_POOL_HPP
#define BEHAVIOR_POOL_BEHAVIOR_POOL_HPP

/* Architecture Messages */
#include<architecture_msgs/Behavior.h>

/* Local Headers */
#include"behavior_manager/behavior_manager.hpp"
#include"behavior_manager/resource_pool.hpp"
#include"behavior_manager/monitored_role.hpp"
#include"behavior_manager/monitored_resource.hpp"

/* C++ Headers */
#include<string>
#include<vector>
#include<map>
#include<type_traits>
#include<stdexcept>
#include<list>

/* ROS Headers */
#include<ros/ros.h>

namespace behavior_manager
{
  template<typename BAHAVIOR>
  class BehaviorPool
  {
  public:
    /**
     * @Default Constructor
     **/
    BehaviorPool() = delete;
    /**
     * @Copy Constructor
     **/
    BehaviorPool(const BehaviorPool&) = delete;
    /**
     * @Move Constructor
     **/
    BehaviorPool(BehaviorPool&&) = delete;
    /**
     * @Constructor
     *
     * @brief
     * After this constructor finishes this object will be ready to make
     * new instances of whatever Behavior Manager it is templating. Note,
     * main has to spin for this object to do it's work.
     *
     * @parameters
     * param_nh: A node handle that points to a ROS namespace that holds
     *           all of this object config information.
     **/
    BehaviorPool(ros::NodeHandle& param_nh);
    /**
     * @Deconstructor
     **/
    ~BehaviorPool() = default;
    /**
     * @Assignment Operators
     **/
    BehaviorPool& operator=(const BehaviorPool&)  = delete;
    BehaviorPool& operator=(      BehaviorPool&&) = delete;
    /**
     * @get
     *
     * @brief
     * Returns the asked for values.
     **/
    const std::string& getName()    const noexcept;
    const std::string& getBaseUri() const noexcept;
  private:
    /* General helper members */
    ros::NodeHandle c_nh;
    /* The name of the behaviors that this object holds */
    std::string name;
    /* URI of the Camunda server */
    std::string base_uri;
    /* Topic names for each Behavior Manager instance */
    std::string status_topic;
    std::string get_resources_topic;
    std::string give_resources_topic;
    std::string update_resources_topic;
    std::string modify_robots_topic;
    /* Roles and resources for the Behavior Managers */
    ResourcePool resources_template;
    /* Listens for new behavior instances */
    ros::Subscriber listen_sub;
    /* Holds behavior instances */
    std::list<BAHAVIOR> behaviors;
    /**
     * @listening_callback
     *
     * @brief
     * Whenever the Behavior Listener publishes a message this callback gets it
     * and if it applies to this behavior either starts a new instance or deletes
     * an old one.
     **/
    void listening_callback(const architecture_msgs::Behavior& msg);
  };

  template<typename BEHAVIOR>
  BehaviorPool<BEHAVIOR>::BehaviorPool(ros::NodeHandle& param_nh)
  {
    static_assert(std::is_base_of<BehaviorManager<>, BEHAVIOR>::value,
                  "BEHAVIOR has to be an inheriting class of BehaviorManager");

    try
    {
      std::string listen_topic;

      // Get config values from ros parameter server
      if(!param_nh.getParam("behavior_name", this->name))
      {
        throw std::runtime_error("didn't find behavior_name in ROS Parameter Server");
      }
      if(!param_nh.getParam("base_uri", this->base_uri))
      {
        throw std::runtime_error("didn't find base_uri in ROS Parameter Server");
      }
      if(!param_nh.getParam("status_topic", this->status_topic))
      {
        throw std::runtime_error("didn't find status_topic in ROS Parameter Server");
      }
      if(!param_nh.getParam("get_resources_topic", this->get_resources_topic))
      {
        throw std::runtime_error("didn't find get_resources_topic in ROS Parameter Server");
      }
      if(!param_nh.getParam("give_resources_topic", this->give_resources_topic))
      {
        throw std::runtime_error("didn't find give_resources_topic in ROS Parameter Server");
      }
      if(!param_nh.getParam("update_resources_topic", this->update_resources_topic))
      {
        throw std::runtime_error("didn't find update_resources_topic in ROS Parameter Server");
      }
      if(!param_nh.getParam("modify_robots_topic", this->modify_robots_topic))
      {
        throw std::runtime_error("didn't find modify_robots_topic in ROS Parameter Server");
      }
      if(!param_nh.getParam("listen_topic", listen_topic))
      {
        throw std::runtime_error("didn't find listen_topic in ROS Parameter Server");
      }

      this->listen_sub = this->c_nh.subscribe(listen_topic, 10, &BehaviorPool::listening_callback, this);

      // Build roles and resources
      if(param_nh.hasParam("role_namespaces"))
      {
        std::vector<std::string> roles;

        if(!param_nh.getParam("role_namespaces", roles))
        {
          throw std::runtime_error("didn't find role_namespaces in ROS Parameter Server");
        }

        for(auto roles_it = roles.begin(); roles_it != roles.end(); roles_it++)
        {
          std::vector<std::string> resource_names;
          MonitoredRole            role;

          if(!param_nh.getParam(*roles_it + "/name", role.getName()))
          {
            throw std::runtime_error("didn't find  " + *roles_it + "/name in ROS Parameter Server");
          }
          if(!param_nh.getParam(*roles_it + "/required", (bool&)role.getRequired()))
          {
            throw std::runtime_error("didn't find " + *roles_it + "/required in ROS Parameter Server");
          }
          if(!param_nh.getParam(*roles_it + "/priority", (int&)role.getPriority()))
          {
            throw std::runtime_error("didn't find role_names in ROS Parameter Server");
          }

          if(!param_nh.getParam(*roles_it + "/resource_namespaces", resource_names))
          {
            throw std::runtime_error("didn't find " + *roles_it + "/resource_namespaces in ROS Parameter Server");
          }

          for(auto resource_it = resource_names.begin(); resource_it != resource_names.end(); resource_it++)
          {
            MonitoredResource& resource = role.resources.emplace(*resource_it, MonitoredResource()).first->second;

            if(!param_nh.getParam(*roles_it + "/" + *resource_it + "/name", resource.getName()))
            {
              throw std::runtime_error("didn't find " + *roles_it + "/" + *resource_it + "/name in ROS Parameter Server");
            }
            if(!param_nh.getParam(*roles_it + "/" + *resource_it + "/category", resource.getCategory()))
            {
              throw std::runtime_error("didn't find " + *roles_it + "/" + *resource_it + "/category in ROS Parameter Server");
            }
            if(!param_nh.getParam(*roles_it + "/" + *resource_it + "/type", resource.getType()))
            {
              throw std::runtime_error("didn't find " + *roles_it + "/" + *resource_it + "/type in ROS Parameter Server");
            }
            if(!param_nh.getParam(*roles_it + "/" + *resource_it + "/required", (bool&)resource.getRequired()))
            {
              throw std::runtime_error("didn't find " + *roles_it + "/" + *resource_it + "/required in ROS Parameter Server");
            }
            if(!param_nh.getParam(*roles_it + "/" + *resource_it + "/priority", (int&)resource.getPriority()))
            {
              throw std::runtime_error("didn't find " + *roles_it + "/" + *resource_it + "/priority in ROS Parameter Server");
            }
            if(!param_nh.getParam(*roles_it + "/" + *resource_it + "/nodes", resource.nodes))
            {
              throw std::runtime_error("didn't find " + *roles_it + "/" + *resource_it + "/nodes in ROS Parameter Server");
            }
          }

          this->resources_template.addRole(std::move(role));
        }
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("BehaviorPool::Constructor error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename BEHAVIOR>
  const std::string& BehaviorPool<BEHAVIOR>::getName() const noexcept
  {
    return this->name;
  }

  template<typename BEHAVIOR>
  const std::string& BehaviorPool<BEHAVIOR>::getBaseUri() const noexcept
  {
    return this->base_uri;
  }

  template<typename BEHAVIOR>
  void BehaviorPool<BEHAVIOR>::listening_callback(const architecture_msgs::Behavior& msg)
  {
    if(this->getName() != msg.name)
    {
      return;
    }

    switch(msg.status)
    {
      case architecture_msgs::Behavior::NEW:
        this->behaviors.emplace_back(this->getBaseUri(),
                                     this->getName(),
                                     msg.topic,
                                     this->status_topic,
                                     this->get_resources_topic,
                                     this->give_resources_topic,
                                     this->update_resources_topic,
                                     this->modify_robots_topic,
                                     this->resources_template);
        this->behaviors.back().unpause();
        break;
      case architecture_msgs::Behavior::CANCELED:
      {
        typename std::list<BEHAVIOR>::iterator to_end = std::find_if(this->behaviors.begin(), this->behaviors.end(),
          [&msg](const BEHAVIOR& ittr)
          {
            return (msg.name + "/" + msg.instance_id == ittr.getName());
          });

        if(to_end != this->behaviors.end())
        {
          to_end->getLock().lock();
          this->behaviors.erase(to_end);
        }
        break;
      }
      default:
        return;
        break;
    }
  }
}// behavior_manager

#endif
/* behavior_pool.hpp */
