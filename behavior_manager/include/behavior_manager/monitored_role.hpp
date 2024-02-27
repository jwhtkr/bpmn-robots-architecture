/**
 * @File: monitored_role.hpp
 * @Date: 16 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * A rapper around an architecture_msgs::Role object that is used to hold additional
 * information and atomize accessing.
 **/

#ifndef BEHAVIOR_MANAGER_MONITORED_ROLE_HPP
#define BEHAVIOR_MANAGER_MONITORED_ROLE_HPP

/* Local Headers */
#include"behavior_manager/monitored_resource.hpp"

/* Architecture Messages */
#include<architecture_msgs/Role.h>

/* C++ Headers */
#include<string>
#include<map>
#include<vector>
#include<memory>

namespace behavior_manager
{
  class MonitoredRole
  {
  public:
    /**
     * @Default Constructor
     **/
    MonitoredRole() = default;
    /**
     * @Copy Constructor
     **/
    MonitoredRole(const MonitoredRole& other) noexcept;
    /**
     * @Move Constructor
     **/
    MonitoredRole(MonitoredRole&& other) noexcept;
    /**
     * @Constructor
     **/
    MonitoredRole(const std::string&                    name,
                  const bool                            required,
                  const uint8_t                         priority,
                  const std::vector<MonitoredResource>& resources) noexcept;

    MonitoredRole(      std::string&&                    name,
                  const bool                             required,
                  const uint8_t                          priority,
                        std::vector<MonitoredResource>&& resources) noexcept;
    /**
     * @Deconstructor
     **/
    ~MonitoredRole() noexcept = default;
    /**
     * @Assignment Operators
     **/
    MonitoredRole& operator=(const MonitoredRole&  other) noexcept;
    MonitoredRole& operator=(      MonitoredRole&& other) noexcept;
    /**
     * @Comparison Operators
     *
     * @brief
     * Compares only based on Role names, resource names.
     **/
    bool operator==(const architecture_msgs::Role& other) const noexcept;
    bool operator!=(const architecture_msgs::Role& other) const noexcept;
    bool operator==(const MonitoredRole&           other) const noexcept;
    bool operator!=(const MonitoredRole&           other) const noexcept;
    /**
     * @hasAll
     *
     * @brief
     * Returns true if all of this Role's Resources are in use.
     **/
    bool hasAll() const noexcept;
    /**
     * @hasRequired
     *
     * @brief
     * Returns true if all of this Role's required Resources are in use.
     **/
    bool hasRequired() const noexcept;
    /**
     * @in_use
     *
     * @brief
     * Returns true if this role holds any resources with in_use == true.
     **/
    bool inUse() const noexcept;
    /**
     * @get
     *
     * @brief
     * Returns the ask for objects.
     **/
    const MonitoredResource&        cgetResource(const std::string& name) const;
          MonitoredResource&         getResource(const std::string& name);
    const std::vector<std::string>& cgetNodes(   const std::string& name) const;
          std::vector<std::string>&  getNodes(   const std::string& name);
    /**
     * @Members
     **/
    /* What resources this behavior uses */
    std::map<std::string,MonitoredResource> resources;
    architecture_msgs::Role                 m_data;
    /* The Role's name */
          std::string&  getName()       noexcept;
    const std::string& cgetName() const noexcept;
    /* Whether or not this Role is required for it's behavior */
          uint8_t&  getRequired()       noexcept;
    const uint8_t& cgetRequired() const noexcept;
    /* How important this Role is to it's behavior */
          uint8_t&  getPriority()       noexcept;
    const uint8_t& cgetPriority() const noexcept;
  };
}// behavior_manager

#endif
/* monitored_resource.hpp */
