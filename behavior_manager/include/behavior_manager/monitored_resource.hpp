/**
 * @File: monitored_resource.hpp
 * @Date: 16 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This object raps a architecture_msgs::Resource object and adds more information
 * holding capabilities.
 **/

#ifndef BEHAVIOR_MANAGER_MONITORED_RESOURCE_HPP
#define BEHAVIOR_MANAGER_MONITORED_RESOURCE_HPP

/* Architecture Messages */
#include<architecture_msgs/Resource.h>

/* C++ Headers */
#include<string>
#include<memory>
#include<vector>

namespace behavior_manager
{
  class MonitoredResource
  {
  public:
    /**
     * @Default Constructor
     **/
    MonitoredResource() noexcept;
    /**
     * @Copy Constructor
     **/
    MonitoredResource(const MonitoredResource& other) noexcept;
    /**
     * @Move Constructor
     **/
    MonitoredResource(MonitoredResource&& other) noexcept = default;
    /**
     * @Constructor
     **/
    MonitoredResource(const architecture_msgs::Resource& resource,
                      const std::vector<std::string>&    nodes) noexcept;

    MonitoredResource(architecture_msgs::Resource&& resource,
                      std::vector<std::string>&&    nodes) noexcept;

    MonitoredResource(const std::string&              name,
                      const std::string&              category,
                      const std::string&              type,
                      const bool                      required,
                      const uint8_t                   priority,
                      const std::vector<std::string>& nodes) noexcept;

    MonitoredResource(      std::string&&              name,
                            std::string&&              category,
                            std::string&&              type,
                      const bool                       required,
                      const uint8_t                    priority,
                            std::vector<std::string>&& nodes) noexcept;
    /**
     * @Deconstructor
     **/
    ~MonitoredResource() noexcept = default;
    /**
     * @Assignment Operators
     **/
    MonitoredResource& operator=(const MonitoredResource&  other) noexcept;
    MonitoredResource& operator=(      MonitoredResource&& other) noexcept = default;
    /**
     * @Comparison Operators
     *
     * @brief
     * Compares only based on the value of name and group name.
     **/
    bool operator==(const architecture_msgs::Resource& other) const noexcept;
    bool operator!=(const architecture_msgs::Resource& other) const noexcept;
    bool operator==(const MonitoredResource&           other) const noexcept;
    bool operator!=(const MonitoredResource&           other) const noexcept;
    /**
     * @Members
     *
     * @brief
     * Most of these are references to the underlying architecture_msgs::Resource object.
     **/
    // Whether or not a robot has been allocated to this resource
    bool in_use;
    // The nodes that are needed to use this resource
    std::vector<std::string> nodes;
    architecture_msgs::Resource m_data;
    // Whether or not it's in use
    bool cinUse() const noexcept;
    // Name of a robot
          std::string&  getGroupName()       noexcept;
    const std::string& cgetGroupName() const noexcept;
    // Name of the resource
          std::string&  getName()       noexcept;
    const std::string& cgetName() const noexcept;
    // What category the resource falls in
          std::string&  getCategory()       noexcept;
    const std::string& cgetCategory() const noexcept;
    // What type of resource this object represents
          std::string&  getType()       noexcept;
    const std::string& cgetType() const noexcept;
    // Whether or not this resource is necessary for the role it belongs to
          uint8_t&  getRequired()       noexcept;
    const uint8_t& cgetRequired() const noexcept;
    // How important this resource is to its role
          uint8_t&  getPriority()       noexcept;
    const uint8_t& cgetPriority() const noexcept;
  };

}// behavior_manager

/* monitored_resource.hpp */
#endif
