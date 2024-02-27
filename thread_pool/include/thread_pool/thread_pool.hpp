/**
 * @File: thread_pool.hpp
 * @Date: 2 August 2019
 * @Author: James Swedeen
 *
 * @brief
 * Class used to spin up nodes.
 **/

#ifndef THREAD_POOL_THREAD_POOL_HPP
#define THREAD_POOL_THREAD_POOL_HPP

/* Local Headers */
#include"thread_pool/data_objects.hpp"

/* Architecture Messages */
#include<architecture_msgs/ModifyRobots.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<map>
#include<mutex>
#include<memory>

namespace thread_pool
{
  class ThreadPool
  {
  public:
    /**
     * @Default Constructor
     **/
    ThreadPool() = delete;
    /**
     * @Copy Constructor
     **/
    ThreadPool(const ThreadPool&) = delete;
    /**
     * @Move Constructor
     **/
    ThreadPool(ThreadPool&&) = delete;
    /**
     * @Constructor
     *
     * @brief
     * Opens ROS topics for use and makes sure the needed services are
     * being provided.
     *
     * @service_provided_topic: The topic that Behavior Managers will use to ask to modify nodes
     * @ros_node_server_start_topic: The name of the start list service that a ROS Node Server
     *                               provides in it's robots namespace
     * @ros_node_server_kill_topic: The name of the kill service that a ROS Node Server provides
     *                              in it's robots namespace
     * @config_file: The absolute path for a Json file holding resource name and node list
     *               mappings
     **/
    ThreadPool(const std::string& service_provided_topic,
               const std::string& ros_node_server_start_topic,
               const std::string& ros_node_server_kill_topic,
               const std::string& config_file);
    /**
     * @Deconstructor
     *
     * @brief
     * Waits for all the lingering service calls to complete.
     **/
    ~ThreadPool() noexcept;
    /**
     * @Assignment Operators
     **/
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool& operator=(ThreadPool&&)      = delete;
    /**
     * @makeThreadPool
     **/
    static std::shared_ptr<ThreadPool> makeThreadPool(const ros::NodeHandle& p_nh);
  private:
    /* General members */
    ros::NodeHandle c_nh;
    /* Used to identify ROS Node Servers on each robot */
    const std::string ros_node_server_start_regex;
    const std::string ros_node_server_kill_regex;
    /* Holds the needed information for starting and ending resources */
    std::map<std::string, ResourceMsgs> resources;
    /* Talks to Behavior Managers */
    ros::ServiceServer provide_srv;
    /* Talks to machine specific ROS Node Servers */
    std::map<std::string, RobotServices> robots_srv;
    /* Protects service calls */
    std::mutex srv_mux;
    /**
     * @setupRobotsCallback
     *
     * @brief
     * For each robot in the request this function will request the appropriate
     * machine remapping node and ROS Node Server to remap topics and start/kill nodes.
     * If this object hasn't found the robot yet it will look for it,
     * throwing if it's not found.
     **/
    bool setupRobotsCallback(architecture_msgs::ModifyRobots::Request&  req,
                             architecture_msgs::ModifyRobots::Response& res);
    /**
     * @findNodeServer
     *
     * @brief
     * Looks for a service that is in the given namespace and has the correct
     * start, and kill regex's. Throws if it can't be found.
     *
     * @name: The name of the robot to look for
     * @return: The services that the requested robot provides
     **/
    RobotServices& findNodeServer(const std::string& name);
    /**
     * @parseConfigFile
     *
     * @brief
     * Uses a JsonPhraser to parse the config file with the passed in absolute path. Throws
     * if file can't be found or the Json held in the file has incorrect formatting.
     *
     * @config_file: Absolute path to the config file
     * @return: All the needed information from the config file. A map of resource names and
     *          data to launch them
     **/
    std::shared_ptr<std::map<std::string, ResourceMsgs>> parseConfigFile(const std::string config_file) const;
  };
}// thread_pool

#endif
/* master_thread_pool.hpp */
