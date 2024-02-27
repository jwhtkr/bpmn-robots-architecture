/**
 * @File: data_objects.hpp
 * @Date: March 2020
 * @Author: James Swedeen
 *
 * @brief
 * This headers holds general struts and enums for the ThreadPool class.
 **/

#ifndef THREAD_POOL_DATA_OBJECTS_HPP
#define THREAD_POOL_DATA_OBJECTS_HPP

/* ROS Node Server Messages */
#include<node_server_msgs/NodeStart.h>
#include<node_server_msgs/Target.h>

/* C++ Headers */
#include<string>
#include<vector>
#include<map>

/* ROS Headers */
#include<ros/ros.h>

namespace thread_pool
{
  struct ResourceMsgs
  {
    /* Resource's name */
    std::string name;
    /* Used to start the needed group of nodes */
    std::vector<node_server_msgs::NodeStart> start;
    /* Used to kill, sleep, and wake up nodes */
    //        robot name
    mutable std::map<std::string, std::vector<node_server_msgs::Target>> targets;
    /**
     * @Default Constructor
     **/
    ResourceMsgs() noexcept = default;
    /**
     * @Constructor
     **/
    explicit ResourceMsgs(const std::string& name) noexcept;
    /**
     * @Copy and Move Constructors
     **/
    ResourceMsgs(const ResourceMsgs&)           = default;
    ResourceMsgs(      ResourceMsgs&&) noexcept = default;
    /**
     * @Deconstructor
     **/
    ~ResourceMsgs() noexcept = default;
    /**
     * @Copy and Move Assignment operators
     **/
    ResourceMsgs& operator=(const ResourceMsgs&)           = default;
    ResourceMsgs& operator=(      ResourceMsgs&&) noexcept = default;
    /**
     * @Comparison Operators
     *
     * @brief
     * Uses this objects name.
     **/
    bool operator==(const ResourceMsgs& rhs) const noexcept;
    bool operator!=(const ResourceMsgs& rhs) const noexcept;
    bool operator>=(const ResourceMsgs& rhs) const noexcept;
    bool operator<=(const ResourceMsgs& rhs) const noexcept;
    bool operator >(const ResourceMsgs& rhs) const noexcept;
    bool operator <(const ResourceMsgs& rhs) const noexcept;
  };

  struct RobotServices
  {
    /* Services Name */
    std::string name;
    /* Used to start a group of nodes */
    ros::ServiceClient starter;
    /* Used to kill nodes */
    ros::ServiceClient ender;
    /**
     * @Default Constructor
     **/
    RobotServices() = default;
    /**
     * @Constructor
     **/
    explicit RobotServices(const std::string& name) noexcept;
    /**
     * @Copy and Move Constructors
     **/
    RobotServices(const RobotServices&)  = default;
    RobotServices(      RobotServices&&) = default;
    /**
     * @Deconstructor
     **/
    ~RobotServices() noexcept = default;
    /**
     * @Copy and Move Assignment operators
     **/
    RobotServices& operator=(const RobotServices&)  noexcept = default;
    RobotServices& operator=(      RobotServices&&) noexcept = default;
    /**
     * @Comparison Operators
     *
     * @brief
     * Uses this objects name.
     **/
    bool operator==(const RobotServices& rhs) const noexcept;
    bool operator!=(const RobotServices& rhs) const noexcept;
    bool operator>=(const RobotServices& rhs) const noexcept;
    bool operator<=(const RobotServices& rhs) const noexcept;
    bool operator >(const RobotServices& rhs) const noexcept;
    bool operator <(const RobotServices& rhs) const noexcept;
 };
}

#endif
/* data_objects.hpp */
